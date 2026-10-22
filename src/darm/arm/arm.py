"""The arm: the only thing in this project that moves the robot.

User interfaces say where they want joints to be. The arm decides what actually
reaches the motors: targets are clamped to limits, slewed at a safe speed, sent
only when they change, and stopped dead the moment the supervisor objects.

Threads: one control loop ticks at config.control_rate_hz. Every public method
may be called from any other thread. A single lock covers each tick end to end,
so a freeze can never interleave with a half-sent setpoint.
"""

import enum
import logging
import math
import threading
import time
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from types import TracebackType

from darm.arm import drives as drive_types
from darm.arm.drives import Drive
from darm.arm.journal import Journal
from darm.arm.supervisor import Supervisor
from darm.config import JointSpec, RobotConfig
from darm.odrive.bus import BusError, CanBus
from darm.odrive.endpoints import EndpointTable
from darm.odrive.node import ODriveNode, SdoTimeoutError
from darm.odrive.protocol import AxisState, ControlMode, InputMode, describe_errors

log = logging.getLogger(__name__)

_ARM_TIMEOUT_S = 2.0
_ARRIVAL_POLL_S = 0.02
_NUDGE_LEAD_S = 0.25
_DISCOVERY_MIN_S = 0.5
_DISCOVERY_POLL_S = 0.01


class ArmState(enum.Enum):
    IDLE = "idle"  # Connected. Motors are not being driven by us.
    ARMED = "armed"  # Holding or moving to targets.
    FROZEN = "frozen"  # Something went wrong. Every joint that can hold is holding; targets are ignored.


class StartupError(RuntimeError):
    pass


class MotionError(RuntimeError):
    """A requested motion could not complete: the arm froze, timed out, or was not armed."""


@dataclass(frozen=True, slots=True)
class Fault:
    reasons: tuple[str, ...]
    at: float

    def __str__(self) -> str:
        return "; ".join(self.reasons)


class Arm:
    def __init__(self, config: RobotConfig, bus: CanBus, journal: Journal | None = None) -> None:
        self.config = config
        self.journal = journal
        endpoints = EndpointTable.for_firmware(config.firmware)
        self.nodes = {node_id: ODriveNode(bus, node_id, endpoints) for node_id in config.node_ids}
        self.drives: list[Drive] = []  # Only drives whose every node answered. Set by discover().
        self._all_drives = [drive_types.build(spec) for spec in config.drives]
        self._supervisor = self._make_supervisor([])

        self._lock = threading.RLock()
        self._state = ArmState.IDLE
        self._fault: Fault | None = None
        self._desired: dict[str, float] = {}  # Where the UI wants each joint.
        self._commanded: dict[str, float] = {}  # Where the arm is currently steering each joint.
        self._sent: dict[int, float] = {}  # Last motor target actually sent, per node.

        self._loop_stop = threading.Event()
        self._loop: threading.Thread | None = None

    # ================================================================ lifecycle

    def discover(self) -> list[int]:
        """Listen for heartbeats; enable every joint whose ODrives all answered. Returns missing node ids.

        Returns as soon as every node has reported both its state and its position. Only a
        missing node costs the full listening window.
        """
        listen_s = max(_DISCOVERY_MIN_S, 2 * self.config.safety.heartbeat_timeout_s)
        deadline = time.monotonic() + listen_s
        while not all(_has_reported(node) for node in self.nodes.values()) and time.monotonic() < deadline:
            time.sleep(_DISCOVERY_POLL_S)
        present = {node_id for node_id, node in self.nodes.items() if _has_reported(node)}
        self.drives = [drive for drive in self._all_drives if set(drive.nodes) <= present]
        if not self.drives:
            raise StartupError("no ODrives answered. Is the CAN interface up and the power box on?")
        self._supervisor = self._make_supervisor(self.drives)
        missing = sorted(set(self.nodes) - present)
        for drive in self._all_drives:
            if drive not in self.drives:
                log.warning("%s disabled: node(s) %s silent", "/".join(drive_types.joint_names([drive])), missing)
        return missing

    def engage(self) -> None:
        """Verify everything, then take control holding every joint exactly where it already is."""
        if not self.drives:
            self.discover()
        self._verify_firmware()
        self._verify_settings()
        self._verify_idle_and_clean()
        self._verify_positions()
        self._verify_zero_reference()
        self._enter_closed_loop(self._active_nodes())
        if self.journal:
            self.journal.mark_away()
        with self._lock:
            self._hold_where_measured()
            self._state = ArmState.ARMED
        self._start_loop()

    def park(self, progress: Callable[[str], None] = log.info) -> None:
        """Fold into the dock one stage at a time, then let go. Blocks. Raises MotionError if anything objects."""
        active = set(self.joint_names)
        for stage in self.config.park.stages:
            joints = [name for name in stage if name in active]
            if joints:
                progress(f"parking {', '.join(joints)}")
                self.move_to(dict.fromkeys(joints, 0.0), self.config.park.stage_timeout_s)
        progress("docked; releasing motors")
        self.release()

    def move_to(self, targets: Mapping[str, float], timeout: float) -> None:
        """Steer joints to targets (clamped to limits) and block until every one has settled there.

        Raises MotionError if the arm is not armed, freezes on the way, or has not arrived
        within timeout seconds, in which case it freezes the arm first.
        """
        with self._lock:
            self._require(ArmState.ARMED, "move")
            for name, value in targets.items():
                self._desired[name] = self.joint(name).clamp(value)
        self._await_arrival(list(targets), timeout)

    def release(self, require_docked: bool = True) -> None:
        """Stop driving the motors. Without brakes the arm then falls, so by default only from the dock."""
        docked = self._is_docked()
        if require_docked and not docked:
            off_dock = {n: v for n, v in self.measured().items() if not abs(v) <= self.joint(n).tolerance}
            raise MotionError(f"not docked: {_format_joints(off_dock, self.joints)}")
        self._stop_loop()
        for node in self._active_nodes():
            node.request_state(AxisState.IDLE)
        with self._lock:
            self._state = ArmState.IDLE
        if self.journal and docked:
            self.journal.mark_docked()

    def close(self) -> None:
        """Stop supervising. Motors keep holding their last target (no watchdog), so nothing drops."""
        self._stop_loop()
        if self.journal and self._state is not ArmState.IDLE and self._is_docked():
            self.journal.mark_docked()

    def __enter__(self) -> Arm:
        return self

    def __exit__(self, kind: type[BaseException] | None, error: BaseException | None, tb: TracebackType | None) -> None:
        self.close()

    # ================================================================ motion

    def set_target(self, joint: str, value: float) -> float:
        """Ask for a joint position. Returns the target actually accepted after clamping to limits."""
        if not math.isfinite(value):
            raise ValueError(f"{joint}: target must be finite")
        spec = self.joint(joint)
        with self._lock:
            if self._state is not ArmState.ARMED:
                return self._desired.get(joint, 0.0)
            self._desired[joint] = spec.clamp(value)
            return self._desired[joint]

    def nudge(self, joint: str, delta: float) -> float:
        """Move a joint's target by delta, but never more than _NUDGE_LEAD_S of travel ahead of the joint.

        Without the bound, key repeat or a stick faster than the joint piles target up
        far ahead, and the joint keeps moving long after the input stops. A nudge may
        stop short; it never moves the target against the direction asked for.
        """
        with self._lock:
            if self._state is not ArmState.ARMED:
                return self._desired.get(joint, 0.0)
            current = self._desired[joint]
            lead = self.joint(joint).max_speed * _NUDGE_LEAD_S
            steering = self._commanded[joint]
            target = current + delta
            if delta > 0:
                target = min(target, max(current, steering + lead))
            elif delta < 0:
                target = max(target, min(current, steering - lead))
            return self.set_target(joint, target)

    def freeze(self, reason: str) -> None:
        with self._lock:
            if self._state is ArmState.ARMED:
                self._freeze((reason,), time.monotonic())

    def clear_fault(self) -> None:
        """Resume after a freeze: re-arm anything that dropped out, then hold where everything is now."""
        with self._lock:
            if self._state is not ArmState.FROZEN:
                return
        nodes = self._active_nodes()
        cleared_at = time.monotonic()
        for node in nodes:
            if node.axis_error:
                node.clear_errors()
        self._enter_closed_loop([n for n in nodes if n.state is not AxisState.CLOSED_LOOP_CONTROL])
        self._await_fresh_heartbeats(nodes, since=cleared_at)
        with self._lock:
            problems = self._supervisor.check(time.monotonic(), expect_closed_loop=True)
            if problems:
                raise StartupError("; ".join(problems))
            self._hold_where_measured()
            self._fault = None
            self._state = ArmState.ARMED
        log.info("fault cleared; holding current position")

    # ================================================================ observation

    @property
    def state(self) -> ArmState:
        return self._state

    @property
    def fault(self) -> Fault | None:
        return self._fault

    @property
    def joint_names(self) -> list[str]:
        return drive_types.joint_names(self.drives)

    @property
    def joints(self) -> dict[str, JointSpec]:
        return drive_types.joint_specs(self.drives)

    def joint(self, name: str) -> JointSpec:
        try:
            return self.joints[name]
        except KeyError:
            raise KeyError(f"no active joint '{name}'; active: {', '.join(self.joint_names)}") from None

    def measured(self) -> dict[str, float]:
        """Joint positions from the latest encoder frames. NaN for a joint with any node missing a frame."""
        positions: dict[str, float] = {}
        for drive in self.drives:
            frames = [self.nodes[n].encoder for n in drive.nodes]
            values = [math.nan] * len(drive.joints)
            if all(frames):
                values = list(drive.to_joint([f.value.pos for f in frames if f]))
            positions.update(zip(drive_types.joint_names([drive]), values, strict=True))
        return positions

    def targets(self) -> dict[str, float]:
        with self._lock:
            return dict(self._desired)

    # ================================================================ control loop

    def _start_loop(self) -> None:
        self._loop_stop.clear()
        self._loop = threading.Thread(target=self._run, name="arm-control", daemon=True)
        self._loop.start()

    def _stop_loop(self) -> None:
        self._loop_stop.set()
        if self._loop is not None and self._loop is not threading.current_thread():
            self._loop.join()
        self._loop = None

    def _run(self) -> None:
        period = 1.0 / self.config.control_rate_hz
        next_tick = time.monotonic()
        while not self._loop_stop.is_set():
            self._tick(time.monotonic(), period)
            next_tick += period
            delay = next_tick - time.monotonic()
            if delay < 0:  # Overran. Resynchronise rather than firing a burst of late ticks.
                next_tick = time.monotonic()
                delay = 0
            self._loop_stop.wait(delay)

    def _tick(self, now: float, dt: float) -> None:
        with self._lock:
            if self._state is ArmState.IDLE:
                return
            armed = self._state is ArmState.ARMED
            commanded = self._commanded if armed else None
            problems = self._supervisor.check(now, expect_closed_loop=True, commanded=commanded)
            if not armed:
                return
            if problems:
                self._freeze(tuple(problems), now)
                return
            try:
                self._advance(dt)
            except BusError as error:
                self._freeze((str(error),), now)

    def _advance(self, dt: float) -> None:
        """Slew each commanded joint toward its desired target, then send whatever changed."""
        for drive in self.drives:
            for joint in drive.joints:
                step = joint.max_speed * dt
                error = self._desired[joint.name] - self._commanded[joint.name]
                self._commanded[joint.name] += max(-step, min(step, error))
            self._send(drive, [self._commanded[j.name] for j in drive.joints])

    def _send(self, drive: Drive, joint_values: list[float]) -> None:
        for node_id, turns in zip(drive.nodes, drive.to_motor(joint_values), strict=True):
            if self._sent.get(node_id) != turns:
                self.nodes[node_id].set_input_pos(turns)
                self._sent[node_id] = turns

    # ================================================================ faults

    def _freeze(self, reasons: tuple[str, ...], now: float) -> None:
        """Bring every joint that still reports in to a controlled stop and hold it there."""
        self._state = ArmState.FROZEN
        self._fault = Fault(reasons, now)
        log.error("FROZEN: %s", self._fault)
        for drive in self.drives:
            stop = self._stopping_point(drive, now)
            if stop is None:
                continue  # Its ODrives stopped reporting. They keep holding their last target on their own.
            for joint, value in zip(drive.joints, stop, strict=True):
                self._desired[joint.name] = self._commanded[joint.name] = value
            try:
                self._send(drive, list(stop))
            except BusError as error:
                log.error("could not send stop to %s: %s", drive.joints[0].name, error)

    def _stopping_point(self, drive: Drive, now: float) -> tuple[float, ...] | None:
        """Where each joint ends up braking at the ODrive's own deceleration limit, clamped to its limits.

        Commanding the current position instead would make a moving joint overshoot
        and swing back, because the ODrive's trajectory planner cannot stop in zero distance.
        """
        stops = []
        for node_id in drive.nodes:
            frame = self.nodes[node_id].encoder
            if frame is None or frame.age(now) > self.config.safety.encoder_timeout_s:
                return None
            pos, vel = frame.value.pos, frame.value.vel
            stops.append(pos + vel * abs(vel) / (2.0 * self.config.decel_limit(node_id)))
        return tuple(joint.clamp(v) for joint, v in zip(drive.joints, drive.to_joint(stops), strict=True))

    # ================================================================ startup checks

    def _verify_firmware(self) -> None:
        wrong = []
        for node in self._active_nodes():
            try:
                firmware = node.request_version().firmware
            except (SdoTimeoutError, BusError) as error:
                raise StartupError(f"node {node.node_id}: {error}") from error
            if firmware != self.config.firmware_base:
                wrong.append(f"node {node.node_id} runs {firmware}")
        if wrong:
            raise StartupError(f"firmware must be {self.config.firmware}: {', '.join(wrong)}")

    def _verify_settings(self) -> None:
        """Two settings that turn a safe arm into a dangerous one if anything ever changes them."""
        problems = []
        for node in self._active_nodes():
            try:
                init_pos = float(node.read("axis0.config.init_pos"))
                watchdog = bool(node.read("axis0.config.enable_watchdog"))
            except (SdoTimeoutError, BusError) as error:
                raise StartupError(f"node {node.node_id}: cannot read settings: {error}") from error
            if not math.isnan(init_pos):
                problems.append(f"node {node.node_id}: init_pos is {init_pos:g}, so arming would lunge there")
            if watchdog:
                problems.append(f"node {node.node_id}: watchdog is on, so a quiet host would drop the arm")
        if problems:
            raise StartupError("; ".join(problems) + ". Run `darm configure`.")

    def _verify_zero_reference(self) -> None:
        """Refuse when the ODrives look freshly powered up but the arm was not left in the dock."""
        if self.journal is None or self.journal.was_docked() is not False:
            return
        if self._is_docked():
            raise StartupError(
                "every joint reads 0, as ODrives do right after power-up, but the arm was last left out of the dock, "
                "so its zero is probably wrong. Power off, seat the arm in the dock, power on, then run `darm home`."
            )

    def _verify_idle_and_clean(self) -> None:
        problems = []
        for node in self._active_nodes():
            if node.axis_error:
                problems.append(f"node {node.node_id}: {describe_errors(node.axis_error)} (run `darm errors --clear`)")
            elif node.state not in (AxisState.IDLE, AxisState.CLOSED_LOOP_CONTROL):
                problems.append(f"node {node.node_id}: busy in {node.state.name}")
        if problems:
            raise StartupError("; ".join(problems))

    def _verify_positions(self) -> None:
        problems = self._supervisor.check(time.monotonic(), expect_closed_loop=False)
        if problems:
            raise StartupError("; ".join(problems) + ". If the arm is in the dock, run `darm home`.")

    def _enter_closed_loop(self, nodes: list[ODriveNode]) -> None:
        """Arm each node holding its measured position, so taking control never moves anything."""
        for node in nodes:
            if node.encoder is None:
                raise StartupError(f"node {node.node_id}: no position feedback")
            node.set_controller_mode(ControlMode.POSITION_CONTROL, InputMode.TRAP_TRAJ)
            node.set_input_pos(node.encoder.value.pos)
            node.request_state(AxisState.CLOSED_LOOP_CONTROL)
        failed = [
            n for n in nodes if not n.wait_until(lambda x: x.state is AxisState.CLOSED_LOOP_CONTROL, _ARM_TIMEOUT_S)
        ]
        if failed:
            details = ", ".join(f"node {n.node_id} ({describe_errors(n.axis_error)})" for n in failed)
            raise StartupError(f"could not enter closed loop: {details}. Armed nodes are holding position.")

    def _await_fresh_heartbeats(self, nodes: list[ODriveNode], since: float) -> None:
        """Judge nodes on what they say after a change, not on a heartbeat that was already in flight."""
        timeout = self.config.safety.heartbeat_timeout_s
        for node in nodes:
            node.wait_until(lambda n: n.heartbeat is not None and n.heartbeat.at > since, timeout)

    def _hold_where_measured(self) -> None:
        """Make the measured position the target, so nothing moves until someone asks it to."""
        for drive in self.drives:
            motor_turns = [self._position(node_id) for node_id in drive.nodes]
            values = [joint.clamp(v) for joint, v in zip(drive.joints, drive.to_joint(motor_turns), strict=True)]
            for joint, value in zip(drive.joints, values, strict=True):
                self._desired[joint.name] = self._commanded[joint.name] = value
            self._sent.update(zip(drive.nodes, motor_turns, strict=True))
            self._send(drive, values)

    # ================================================================ helpers

    def _await_arrival(self, joints: list[str], timeout: float) -> None:
        deadline = time.monotonic() + timeout
        while True:
            with self._lock:
                self._require(ArmState.ARMED, "move")
                arrived = all(self._commanded[name] == self._desired[name] for name in joints)
            measured = self.measured()
            close = all(abs(measured[n] - self._desired[n]) <= self.joint(n).tolerance for n in joints)
            if arrived and close and self._trajectories_done(joints):
                return
            if time.monotonic() > deadline:
                self.freeze(f"{', '.join(joints)} did not reach target within {timeout:g} s")
                self._require(ArmState.ARMED, "move")
            time.sleep(_ARRIVAL_POLL_S)

    def _position(self, node_id: int) -> float:
        frame = self.nodes[node_id].encoder
        if frame is None:
            raise StartupError(f"node {node_id}: no position feedback")
        return frame.value.pos

    def _is_docked(self) -> bool:
        return all(abs(value) <= self.joint(name).tolerance for name, value in self.measured().items())

    def _trajectories_done(self, joints: list[str]) -> bool:
        names = set(joints)
        nodes = [self.nodes[n] for d in self.drives if names & {j.name for j in d.joints} for n in d.nodes]
        return all(node.heartbeat is not None and node.heartbeat.value.trajectory_done for node in nodes)

    def _require(self, state: ArmState, action: str) -> None:
        if self._state is not state:
            reason = f": {self._fault}" if self._fault else ""
            raise MotionError(f"cannot {action} while {self._state.value}{reason}")

    def _make_supervisor(self, drives: list[Drive]) -> Supervisor:
        limits = {n: limit for n in self.nodes if (limit := self.config.current_limit(n)) is not None}
        return Supervisor(self.config.safety, drives, self.nodes, limits)

    def _active_nodes(self) -> list[ODriveNode]:
        return [self.nodes[node_id] for drive in self.drives for node_id in drive.nodes]


def _has_reported(node: ODriveNode) -> bool:
    return node.heartbeat is not None and node.encoder is not None


def _format_joints(values: Mapping[str, float], specs: Mapping[str, JointSpec]) -> str:
    return ", ".join(f"{name} {value:.1f} {specs[name].unit}" for name, value in values.items())
