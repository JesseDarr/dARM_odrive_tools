"""One ODrive on the bus: its latest telemetry and the commands it accepts.

Telemetry arrives on the bus reader thread and is stored as immutable snapshots.
Rebinding an attribute is atomic in CPython, so any thread reading node.encoder
gets a complete message from one frame without taking a lock.
"""

from __future__ import annotations

import logging
import struct
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import Generic, TypeVar

from darm.odrive import protocol as p
from darm.odrive.bus import CanBus
from darm.odrive.endpoints import Endpoint, EndpointTable

log = logging.getLogger(__name__)

T = TypeVar("T")

DEFAULT_SDO_TIMEOUT_S = 0.1
_POLL_INTERVAL_S = 0.01


@dataclass(frozen=True, slots=True)
class Stamped(Generic[T]):
    value: T
    at: float  # time.monotonic() when the frame arrived

    def age(self, now: float) -> float:
        return now - self.at


class SdoTimeoutError(TimeoutError):
    pass


class _SdoChannel:
    """At most one endpoint request in flight per node; replies matched on endpoint id and size.

    Since firmware 0.6.11 every write is acknowledged with an empty TxSdo. A late
    ack, or a reply to a request that already timed out, must never be taken as
    the answer to the next question. Replies are accepted only when they name the
    endpoint being waited on and carry at least as many bytes as that answer needs.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._arrived = threading.Event()
        self._awaiting: tuple[int, int] | None = None  # (endpoint id, minimum value bytes)
        self._reply = b""

    def request(self, send: Callable[[], None], endpoint_id: int, min_bytes: int, timeout: float) -> bytes:
        with self._lock:
            self._arrived.clear()
            self._awaiting = (endpoint_id, min_bytes)
            try:
                send()
                if not self._arrived.wait(timeout):
                    raise SdoTimeoutError(f"no reply for endpoint {endpoint_id} within {timeout * 1000:.0f} ms")
                return self._reply
            finally:
                self._awaiting = None

    def on_reply(self, data: bytes) -> None:
        endpoint_id, value = p.decode_sdo_response(data)
        awaiting = self._awaiting
        if awaiting is not None and endpoint_id == awaiting[0] and len(value) >= awaiting[1]:
            self._reply = value
            self._arrived.set()


class ODriveNode:
    def __init__(self, bus: CanBus, node_id: int, endpoints: EndpointTable | None = None) -> None:
        self.node_id = node_id
        self.endpoints = endpoints
        self._bus = bus
        self._sdo = _SdoChannel()

        self.heartbeat: Stamped[p.Heartbeat] | None = None
        self.errors: Stamped[p.Errors] | None = None
        self.encoder: Stamped[p.EncoderEstimates] | None = None
        self.iq: Stamped[p.Iq] | None = None
        self.temperature: Stamped[p.Temperature] | None = None
        self.bus_power: Stamped[p.BusVoltageCurrent] | None = None
        self.torques: Stamped[p.Torques] | None = None
        self.powers: Stamped[p.Powers] | None = None
        self.version: Stamped[p.Version] | None = None

        bus.subscribe(node_id, self._on_frame)

    def __repr__(self) -> str:
        return f"ODriveNode({self.node_id})"

    # ------------------------------------------------------------ state shortcuts

    @property
    def state(self) -> p.AxisState:
        return self.heartbeat.value.axis_state if self.heartbeat else p.AxisState.UNDEFINED

    @property
    def axis_error(self) -> int:
        return self.heartbeat.value.axis_error if self.heartbeat else 0

    # ------------------------------------------------------------ commands (fire and forget)

    def request_state(self, state: p.AxisState) -> None:
        self._send(p.Cmd.SET_AXIS_STATE, p.encode_set_axis_state(state))

    def set_controller_mode(self, control: p.ControlMode, inp: p.InputMode) -> None:
        self._send(p.Cmd.SET_CONTROLLER_MODE, p.encode_set_controller_mode(control, inp))

    def set_input_pos(self, turns: float) -> None:
        self._send(p.Cmd.SET_INPUT_POS, p.encode_set_input_pos(turns))

    def set_input_vel(self, turns_per_s: float) -> None:
        self._send(p.Cmd.SET_INPUT_VEL, p.encode_set_input_vel(turns_per_s))

    def set_absolute_position(self, turns: float) -> None:
        self._send(p.Cmd.SET_ABSOLUTE_POSITION, p.encode_set_absolute_position(turns))

    def clear_errors(self, identify: bool = False) -> None:
        """Clear latched errors. identify=True also blinks the LED until cleared again without it."""
        self._send(p.Cmd.CLEAR_ERRORS, p.encode_clear_errors(identify))

    def reboot(self, action: p.RebootAction = p.RebootAction.REBOOT) -> None:
        self._send(p.Cmd.REBOOT, p.encode_reboot(action))

    # ------------------------------------------------------------ queries (block until answered)

    def request_version(self, timeout: float = DEFAULT_SDO_TIMEOUT_S) -> p.Version:
        return self._poll(p.Cmd.GET_VERSION, lambda node: node.version, timeout)

    def request_errors(self, timeout: float = DEFAULT_SDO_TIMEOUT_S) -> p.Errors:
        return self._poll(p.Cmd.GET_ERROR, lambda node: node.errors, timeout)

    def read(self, path: str, timeout: float = DEFAULT_SDO_TIMEOUT_S) -> bool | int | float:
        endpoint = self._endpoint(path)
        request = p.encode_sdo_read(endpoint.id)
        reply = self._sdo.request(lambda: self._send(p.Cmd.RX_SDO, request), endpoint.id, endpoint.size, timeout)
        return endpoint.decode(reply)

    def write(self, path: str, value: bool | int | float, timeout: float = DEFAULT_SDO_TIMEOUT_S) -> None:
        """Write and wait for the firmware's acknowledgement (0.6.11+). Raises SdoTimeoutError if none comes."""
        endpoint = self._endpoint(path)
        if endpoint.is_read_only:
            raise PermissionError(f"{path} is read-only")
        request = p.encode_sdo_write(endpoint.id, endpoint.encode(value))
        self._sdo.request(lambda: self._send(p.Cmd.RX_SDO, request), endpoint.id, 0, timeout)

    def wait_until(self, condition: Callable[[ODriveNode], bool], timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while not condition(self):
            if time.monotonic() >= deadline:
                return False
            time.sleep(_POLL_INTERVAL_S)
        return True

    # ------------------------------------------------------------ internals

    def _poll(self, cmd: p.Cmd, latest: Callable[[ODriveNode], Stamped[T] | None], timeout: float) -> T:
        """Ask for one Get_* message with a remote frame and wait for a reply newer than the question."""
        asked_at = time.monotonic()
        self._bus.send(self.node_id, cmd, remote=True)

        def answered(node: ODriveNode) -> bool:
            stamped = latest(node)
            return stamped is not None and stamped.at >= asked_at

        if not self.wait_until(answered, timeout):
            raise SdoTimeoutError(f"node {self.node_id} did not answer {cmd.name}")
        stamped = latest(self)
        assert stamped is not None
        return stamped.value

    def _endpoint(self, path: str) -> Endpoint:
        if self.endpoints is None:
            raise RuntimeError(f"node {self.node_id} has no endpoint table; endpoint access needs one")
        return self.endpoints[path]

    def _send(self, cmd: p.Cmd, data: bytes) -> None:
        self._bus.send(self.node_id, cmd, data)

    def _on_frame(self, cmd: int, data: bytes, now: float) -> None:
        try:
            self._store(cmd, data, now)
        except struct.error:
            log.warning("node %d: malformed frame cmd 0x%02X (%d bytes)", self.node_id, cmd, len(data))

    def _store(self, cmd: int, data: bytes, now: float) -> None:
        match cmd:
            case p.Cmd.HEARTBEAT:
                self.heartbeat = Stamped(p.decode_heartbeat(data), now)
            case p.Cmd.GET_ENCODER_ESTIMATES:
                self.encoder = Stamped(p.decode_encoder_estimates(data), now)
            case p.Cmd.GET_ERROR:
                self.errors = Stamped(p.decode_errors(data), now)
            case p.Cmd.GET_IQ:
                self.iq = Stamped(p.decode_iq(data), now)
            case p.Cmd.GET_TEMPERATURE:
                self.temperature = Stamped(p.decode_temperature(data), now)
            case p.Cmd.GET_BUS_VOLTAGE_CURRENT:
                self.bus_power = Stamped(p.decode_bus_voltage_current(data), now)
            case p.Cmd.GET_TORQUES:
                self.torques = Stamped(p.decode_torques(data), now)
            case p.Cmd.GET_POWERS:
                self.powers = Stamped(p.decode_powers(data), now)
            case p.Cmd.GET_VERSION:
                self.version = Stamped(p.decode_version(data), now)
            case p.Cmd.TX_SDO:
                self._sdo.on_reply(data)
