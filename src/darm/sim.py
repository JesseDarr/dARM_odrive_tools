"""Simulated ODrives speaking CANSimple on a python-can virtual bus.

Enough to exercise every code path in this project without hardware: cyclic
telemetry, axis states, trajectory-limited motion toward input_pos, endpoint
reads and writes, errors, version, reboot, and Address-based discovery of
freshly flashed (unaddressed) ODrives. It is not a physics model. Joints do not
sag under gravity; tests inject faults explicitly.

One thread runs the whole network, so a simulation step is deterministic.
"""

from __future__ import annotations

import contextlib
import math
import struct
import threading
import time
from collections.abc import Callable, Iterable, Mapping
from dataclasses import dataclass, field
from types import TracebackType

import can

from darm.config import RobotConfig
from darm.odrive import protocol as p
from darm.odrive.endpoints import EndpointTable

_STEP_S = 0.002
_REBOOT_SILENCE_S = 0.5  # A real S1 is gone for well over a few heartbeats while it saves and reboots.
_CALIBRATION_S = 0.5
_SERIAL_BASE = 0x3A57_0000_0000
_PROTOCOL_VERSION = 2
_HARDWARE = (5, 2, 0)  # ODrive S1

_TELEMETRY_KEYS = {
    "heartbeat_ms": p.Cmd.HEARTBEAT,
    "encoder_ms": p.Cmd.GET_ENCODER_ESTIMATES,
    "error_ms": p.Cmd.GET_ERROR,
    "iq_ms": p.Cmd.GET_IQ,
    "torques_ms": p.Cmd.GET_TORQUES,
    "bus_voltage_ms": p.Cmd.GET_BUS_VOLTAGE_CURRENT,
    "temperature_ms": p.Cmd.GET_TEMPERATURE,
    "powers_ms": p.Cmd.GET_POWERS,
}


def simulated_serial(node_id: int) -> int:
    """The serial number the simulator gives the ODrive that the config calls node_id."""
    return _SERIAL_BASE + node_id


@dataclass
class SimNode:
    node_id: int  # BROADCAST_NODE_ID while unaddressed
    firmware: tuple[int, int, int]
    serial_number: int = 0
    vel_limit: float = 5.0  # turns/s
    accel: float = 8.0  # turns/s^2

    state: p.AxisState = p.AxisState.IDLE
    axis_error: int = 0
    disarm_reason: int = 0
    procedure_result: p.ProcedureResult = p.ProcedureResult.SUCCESS
    pos: float = 0.0  # Where the motor is.
    vel: float = 0.0
    setpoint: float = 0.0  # Where the trajectory planner says it should be.
    setpoint_vel: float = 0.0
    input_pos: float = 0.0  # Where the host asked it to go.
    fet_temp: float = 35.0
    iq: float = 0.0
    bus_voltage: float = 48.0
    silent: bool = False
    blocked: bool = False
    identifying: bool = False  # LED blinking, as asked by Clear_Errors with its identify byte set.
    calibration_result: p.ProcedureResult = p.ProcedureResult.SUCCESS
    values: dict[int, bytes] = field(default_factory=dict)
    saved: bool = False

    _busy_until: float = 0.0
    _silent_until: float = 0.0

    @property
    def addressed(self) -> bool:
        return self.node_id != p.BROADCAST_NODE_ID

    @property
    def trajectory_done(self) -> bool:
        """Like the real firmware: the planner finished, whether or not the motor actually got there."""
        return self.setpoint == self.input_pos and self.setpoint_vel == 0.0

    def teleport(self, turns: float) -> None:
        self.pos = self.setpoint = self.input_pos = turns
        self.vel = self.setpoint_vel = 0.0

    def fault(self, error: p.ODriveError) -> None:
        """What a real ODrive does on most errors: latch it and disarm."""
        self.axis_error |= error
        self.disarm_reason |= error
        self.state = p.AxisState.IDLE
        self.vel = 0.0

    def step(self, now: float, dt: float) -> None:
        if self.state is p.AxisState.FULL_CALIBRATION_SEQUENCE and now >= self._busy_until:
            self.state = p.AxisState.IDLE
            self.procedure_result = self.calibration_result
        if self.state is not p.AxisState.CLOSED_LOOP_CONTROL:
            self.vel = self.setpoint_vel = 0.0
            self.setpoint = self.input_pos = self.pos
            return
        self._advance_trajectory(dt)
        if self.blocked:
            self.vel = 0.0
        else:
            self.pos, self.vel = self.setpoint, self.setpoint_vel

    def _advance_trajectory(self, dt: float) -> None:
        error = self.input_pos - self.setpoint
        if abs(error) < 1e-6 and abs(self.setpoint_vel) < 1e-3:
            self.setpoint, self.setpoint_vel = self.input_pos, 0.0
            return
        braking_speed = math.sqrt(2.0 * self.accel * abs(error))
        target_vel = math.copysign(min(self.vel_limit, braking_speed), error)
        self.setpoint_vel += max(-self.accel * dt, min(self.accel * dt, target_vel - self.setpoint_vel))
        step = self.setpoint_vel * dt
        if abs(step) >= abs(error):
            self.setpoint, self.setpoint_vel = self.input_pos, 0.0
        else:
            self.setpoint += step


_TELEMETRY_ENCODERS: dict[int, Callable[[SimNode], bytes]] = {
    p.Cmd.HEARTBEAT: lambda n: struct.pack("<IBBB", n.axis_error, n.state, n.procedure_result, n.trajectory_done),
    p.Cmd.GET_ENCODER_ESTIMATES: lambda n: struct.pack("<ff", n.pos, n.vel),
    p.Cmd.GET_ERROR: lambda n: struct.pack("<II", n.axis_error, n.disarm_reason),
    p.Cmd.GET_BUS_VOLTAGE_CURRENT: lambda n: struct.pack("<ff", n.bus_voltage, 0.5),
    p.Cmd.GET_TEMPERATURE: lambda n: struct.pack("<ff", n.fet_temp, math.nan),
    p.Cmd.GET_IQ: lambda n: struct.pack("<ff", n.iq, n.iq),
}


class SimNetwork:
    """Every node in a RobotConfig, on one virtual CAN channel.

    freshly_flashed=True starts every ODrive the way a firmware update leaves it:
    unaddressed, unconfigured, silent except for the Address message.
    """

    def __init__(
        self,
        config: RobotConfig,
        channel: str = "darm-sim",
        node_ids: Iterable[int] | None = None,
        freshly_flashed: bool = False,
    ) -> None:
        self._config = config
        self._endpoints = EndpointTable.for_firmware(config.firmware)
        major, minor, revision = (int(x) for x in config.firmware_base.split("."))
        ids = config.node_ids if node_ids is None else tuple(node_ids)
        self._nodes = [self._make_node(node_id, (major, minor, revision), freshly_flashed) for node_id in ids]

        rates = config.telemetry_ms
        self._periods = {cmd: rates[key] / 1000 for key, cmd in _TELEMETRY_KEYS.items() if rates[key] > 0}
        self._next_due: dict[tuple[int, int], float] = {}
        self._bus = can.Bus(interface="virtual", channel=channel, receive_own_messages=False)
        self._lock = threading.RLock()  # Re-entrant: a test that nests locked() must not hang the run.
        self._running = True
        self._thread = threading.Thread(target=self._run, name="sim-odrives", daemon=True)
        self._thread.start()

    @property
    def nodes(self) -> dict[int, SimNode]:
        """Addressed nodes by id. An unaddressed node has no id to find it by; see all_nodes."""
        return {node.node_id: node for node in self._nodes if node.addressed}

    @property
    def all_nodes(self) -> list[SimNode]:
        return list(self._nodes)

    def _make_node(self, node_id: int, firmware: tuple[int, int, int], freshly_flashed: bool) -> SimNode:
        node = SimNode(node_id, firmware, serial_number=simulated_serial(node_id))
        if node_id in self._config.node_ids:
            node.accel = self._config.decel_limit(node_id)
            if not freshly_flashed:
                node.values = self._configured_values(node_id)
        if freshly_flashed:
            node.node_id = p.BROADCAST_NODE_ID
        return node

    def _configured_values(self, node_id: int) -> dict[int, bytes]:
        """Endpoint values as `darm configure` would have left them."""
        settings = self._config.settings_for(node_id).items()
        return {self._endpoints[path].id: self._endpoints[path].encode(value) for path, value in settings}

    def close(self) -> None:
        self._running = False
        self._thread.join(timeout=1.0)
        self._bus.shutdown()

    def __enter__(self) -> SimNetwork:
        return self

    def __exit__(self, kind: type[BaseException] | None, error: BaseException | None, tb: TracebackType | None) -> None:
        self.close()

    def locked(self) -> threading.RLock:
        """Hold this while poking node state from a test, so a step never sees half a change."""
        return self._lock

    # ------------------------------------------------------------ loop

    def _run(self) -> None:
        last = time.monotonic()
        while self._running:
            now = time.monotonic()
            with self._lock:
                self._receive_all(now)
                for node in self._nodes:
                    node.step(now, now - last)
                self._emit_telemetry(now)
            last = now
            time.sleep(_STEP_S)

    def _receive_all(self, now: float) -> None:
        while (message := self._bus.recv(timeout=0)) is not None:
            node_id, cmd = p.split_arbitration_id(message.arbitration_id)
            for node in self._recipients(node_id):
                if node.silent or now < node._silent_until:
                    continue
                with contextlib.suppress(struct.error, IndexError, ValueError):  # Malformed frames are ignored.
                    self._handle(node, cmd, bytes(message.data), message.is_remote_frame, now)

    def _recipients(self, node_id: int) -> list[SimNode]:
        if node_id == p.BROADCAST_NODE_ID:
            return list(self._nodes)
        return [node for node in self._nodes if node.node_id == node_id]

    def _emit_telemetry(self, now: float) -> None:
        for node in self._nodes:
            if not node.addressed or node.silent or now < node._silent_until:
                continue
            for cmd, period in self._periods.items():
                key = (node.node_id, cmd)
                if now >= self._next_due.get(key, 0.0):
                    self._next_due[key] = now + period
                    self._send(node, cmd, self._telemetry(node, cmd))

    def _telemetry(self, node: SimNode, cmd: int) -> bytes:
        encode = _TELEMETRY_ENCODERS.get(cmd)
        return encode(node) if encode else struct.pack("<ff", 0.0, 0.0)

    # ------------------------------------------------------------ commands

    def _handle(self, node: SimNode, cmd: int, data: bytes, remote: bool, now: float) -> None:
        if cmd == p.Cmd.ADDRESS:
            self._handle_address(node, data, remote)
        elif remote:
            self._answer_remote(node, cmd)
        else:
            self._handle_command(node, cmd, data, now)

    def _answer_remote(self, node: SimNode, cmd: int) -> None:
        if cmd == p.Cmd.GET_VERSION:
            self._send(node, cmd, bytes([_PROTOCOL_VERSION, *_HARDWARE, *node.firmware, 0]))
        elif (encode := _TELEMETRY_ENCODERS.get(cmd)) is not None:
            self._send(node, cmd, encode(node))

    def _handle_address(self, node: SimNode, data: bytes, remote: bool) -> None:
        """Discovery and addressing, by the rules in the CAN protocol's Address message."""
        if remote or not data:
            self._send(node, p.Cmd.ADDRESS, p.encode_address(node.node_id, node.serial_number) + b"\x00")
            return
        request = p.decode_address(data)
        if request.serial_number in (0, node.serial_number):
            node.node_id = request.node_id
        elif request.node_id == node.node_id:
            node.node_id = p.BROADCAST_NODE_ID  # Someone else is taking this id.

    def _handle_command(self, node: SimNode, cmd: int, data: bytes, now: float) -> None:
        match cmd:
            case p.Cmd.SET_AXIS_STATE:
                self._set_state(node, p.AxisState(struct.unpack_from("<I", data)[0]), now)
            case p.Cmd.SET_INPUT_POS:
                node.input_pos = struct.unpack_from("<f", data)[0]
            case p.Cmd.SET_ABSOLUTE_POSITION:
                node.teleport(struct.unpack_from("<f", data)[0])
            case p.Cmd.CLEAR_ERRORS:
                node.axis_error = node.disarm_reason = 0
                node.identifying = bool(data and data[0])
            case p.Cmd.ESTOP:
                node.fault(p.ODriveError.ESTOP_REQUESTED)
            case p.Cmd.REBOOT:
                node.saved |= bool(data) and data[0] == p.RebootAction.SAVE_CONFIGURATION
                node.state, node.vel = p.AxisState.IDLE, 0.0
                node.identifying = False
                node._silent_until = now + _REBOOT_SILENCE_S
            case p.Cmd.RX_SDO:
                self._handle_sdo(node, data)

    def _set_state(self, node: SimNode, state: p.AxisState, now: float) -> None:
        if state is p.AxisState.CLOSED_LOOP_CONTROL:
            if node.axis_error:
                return
            node.setpoint = node.input_pos = node.pos
            node.state = state
        elif state is p.AxisState.FULL_CALIBRATION_SEQUENCE:
            node.state = state
            node.procedure_result = p.ProcedureResult.BUSY
            node._busy_until = now + _CALIBRATION_S
        else:
            node.state = state

    def _handle_sdo(self, node: SimNode, data: bytes) -> None:
        opcode, endpoint_id, _ = struct.unpack_from("<BHB", data)
        if opcode == 1:  # Write. Firmware 0.6.11+ acknowledges with an empty reply.
            node.values[endpoint_id] = bytes(data[4:])
            self._send(node, p.Cmd.TX_SDO, struct.pack("<BHB", 0, endpoint_id, 0))
            return
        value = node.values.get(endpoint_id, bytes(4))
        self._send(node, p.Cmd.TX_SDO, struct.pack("<BHB", 0, endpoint_id, 0) + value)

    def _send(self, node: SimNode, cmd: int, data: bytes) -> None:
        if not node.addressed and cmd != p.Cmd.ADDRESS:
            return  # An unaddressed ODrive answers nothing but Address.
        self._bus.send(can.Message(arbitration_id=p.arbitration_id(node.node_id, cmd), data=data, is_extended_id=False))


def place(network: SimNetwork, motor_turns: Mapping[int, float]) -> None:
    """Teleport nodes to positions, as if the arm had been powered on there."""
    with network.locked():
        for node_id, turns in motor_turns.items():
            network.nodes[node_id].teleport(turns)
