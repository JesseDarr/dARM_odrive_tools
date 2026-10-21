"""Robot description: darm.toml -> validated, immutable objects.

Every mistake a config file can contain is caught here, before anything talks
to the bus. Unknown keys are errors too: a misspelt limit that silently falls
back to a default is exactly the kind of bug that moves an arm somewhere new.
"""

from __future__ import annotations

import math
import sys
from collections.abc import Iterable, Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal, TypeVar, cast

from darm.odrive.endpoints import EndpointTable
from darm.odrive.protocol import MAX_NODE_ID, ControlMode, InputMode, MotorType, base_version

if sys.version_info >= (3, 11):
    import tomllib
else:
    import tomli as tomllib

SettingValue = bool | int | float
Unit = Literal["deg", "mm"]
DriveKind = Literal["direct", "differential"]
Shift = Literal["always", "shifted", "unshifted"]

_DEGREES_PER_TURN = 360.0
_ENUM_SETTINGS = {
    "axis0.config.motor.motor_type": MotorType,
    "axis0.controller.config.control_mode": ControlMode,
    "axis0.controller.config.input_mode": InputMode,
}
_TELEMETRY_ENDPOINTS = {
    "heartbeat_ms": "axis0.config.can.heartbeat_msg_rate_ms",
    "encoder_ms": "axis0.config.can.encoder_msg_rate_ms",
    "error_ms": "axis0.config.can.error_msg_rate_ms",
    "iq_ms": "axis0.config.can.iq_msg_rate_ms",
    "torques_ms": "axis0.config.can.torques_msg_rate_ms",
    "bus_voltage_ms": "axis0.config.can.bus_voltage_msg_rate_ms",
    "temperature_ms": "axis0.config.can.temperature_msg_rate_ms",
    "powers_ms": "axis0.config.can.powers_msg_rate_ms",
}
DECEL_LIMIT_SETTING = "axis0.trap_traj.config.decel_limit"
CURRENT_LIMIT_SETTING = "axis0.config.motor.current_soft_max"
MOTOR_TYPE_SETTING = "axis0.config.motor.motor_type"


class ConfigError(ValueError):
    pass


@dataclass(frozen=True, slots=True)
class JointSpec:
    name: str
    unit: Unit
    min: float
    max: float
    max_speed: float  # units/s
    tolerance: float  # units

    def clamp(self, value: float) -> float:
        return min(self.max, max(self.min, value))


@dataclass(frozen=True, slots=True)
class DriveSpec:
    """How motors become joint motion.

    direct:       one joint driven by one or more motors in lockstep.
    differential: two motors driving two joints (bend, twist).
    """

    kind: DriveKind
    motor: str
    nodes: tuple[int, ...]
    signs: tuple[int, ...]
    turns_per_unit: float  # motor turns per joint unit
    joints: tuple[JointSpec, ...]  # differential: (bend, twist)


@dataclass(frozen=True, slots=True)
class BusSpec:
    interface: str
    channel: str
    bitrate: int


@dataclass(frozen=True, slots=True)
class SafetySpec:
    heartbeat_timeout_s: float
    encoder_timeout_s: float
    max_fet_temp_c: float
    min_bus_voltage: float
    max_bus_voltage: float
    max_current_fraction: float  # Of current_soft_max, for motors whose limit is in amps.
    transient_s: float


@dataclass(frozen=True, slots=True)
class ParkSpec:
    stages: tuple[tuple[str, ...], ...]
    stage_timeout_s: float


@dataclass(frozen=True, slots=True)
class GamepadBinding:
    axis: int
    joint: str
    speed: float  # joint units/s at full deflection
    when: Shift
    trigger: bool


@dataclass(frozen=True, slots=True)
class GamepadSpec:
    deadman_button: int
    shift_button: int
    dead_zone: float
    rate_hz: float
    bindings: tuple[GamepadBinding, ...]


@dataclass(frozen=True, slots=True)
class RobotConfig:
    firmware: str
    control_rate_hz: float
    bus: BusSpec
    telemetry_ms: Mapping[str, int]
    safety: SafetySpec
    park: ParkSpec
    drives: tuple[DriveSpec, ...]
    gamepad: GamepadSpec
    common_settings: Mapping[str, SettingValue]
    motors: Mapping[str, Mapping[str, SettingValue]]

    @property
    def joints(self) -> dict[str, JointSpec]:
        return {joint.name: joint for drive in self.drives for joint in drive.joints}

    @property
    def node_ids(self) -> tuple[int, ...]:
        return tuple(sorted(node for drive in self.drives for node in drive.nodes))

    def drive_for_node(self, node_id: int) -> DriveSpec:
        return next(drive for drive in self.drives if node_id in drive.nodes)

    def settings_for(self, node_id: int) -> dict[str, SettingValue]:
        """Every endpoint value `darm configure` writes to this node, in write order."""
        telemetry = {_TELEMETRY_ENDPOINTS[key]: ms for key, ms in self.telemetry_ms.items()}
        return {**self.common_settings, **telemetry, **self.motors[self.drive_for_node(node_id).motor]}

    def decel_limit(self, node_id: int) -> float:
        """The ODrive's trajectory deceleration in motor turns/s^2: how hard it can stop."""
        return float(self.motors[self.drive_for_node(node_id).motor][DECEL_LIMIT_SETTING])

    @property
    def firmware_base(self) -> str:
        """What the ODrives report over CAN: '0.6.12-1' -> '0.6.12'."""
        return base_version(self.firmware)

    def current_limit(self, node_id: int) -> float | None:
        """current_soft_max in amps for current-controlled motors.

        None for voltage-controlled (gimbal) motors: they have no current sensing,
        so their Iq is a model estimate, and the gripper is meant to squeeze anyway.
        """
        profile = self.motors[self.drive_for_node(node_id).motor]
        if profile.get(MOTOR_TYPE_SETTING) != MotorType.HIGH_CURRENT or CURRENT_LIMIT_SETTING not in profile:
            return None
        return float(profile[CURRENT_LIMIT_SETTING])


def load(path: Path) -> RobotConfig:
    try:
        raw = tomllib.loads(path.read_text(encoding="utf-8"))
    except (OSError, tomllib.TOMLDecodeError) as error:
        raise ConfigError(f"{path}: {error}") from error
    config = _parse(_Section(raw, path.name))
    check_endpoints(config, EndpointTable.for_firmware(config.firmware))
    return config


def check_endpoints(config: RobotConfig, table: EndpointTable) -> None:
    """Every setting must name a real, writable endpoint in this firmware."""
    for node_id in config.node_ids:
        for path in config.settings_for(node_id):
            endpoint = table[path]  # Raises with "did you mean" suggestions.
            if endpoint.is_function or endpoint.is_read_only:
                kind = "a function" if endpoint.is_function else "read-only"
                raise ConfigError(f"'{path}' cannot be set: it is {kind}")


# ------------------------------------------------------------------------------ parsing

T = TypeVar("T")
_REQUIRED: Any = object()


class _Section:
    """A TOML table that type-checks every read and rejects keys nobody read."""

    def __init__(self, data: Mapping[str, Any], where: str) -> None:
        self._data = data
        self._where = where
        self._used: set[str] = set()

    def get(self, key: str, kind: type[T], default: T = _REQUIRED) -> T:
        self._used.add(key)
        if key not in self._data:
            if default is _REQUIRED:
                raise ConfigError(f"{self._where}: missing '{key}'")
            return default
        value = self._data[key]
        if kind is float and isinstance(value, int) and not isinstance(value, bool):
            value = float(value)
        if not isinstance(value, kind) or (kind is int and isinstance(value, bool)):
            raise ConfigError(f"{self._where}: '{key}' must be {kind.__name__}, got {value!r}")
        if isinstance(value, float) and not math.isfinite(value):
            raise ConfigError(f"{self._where}: '{key}' must be finite")
        return value

    def section(self, key: str) -> _Section:
        return _Section(self.get(key, dict), f"{self._where}.{key}")

    def sections(self, key: str) -> list[_Section]:
        items = self.get(key, list, [])
        return [_Section(item, f"{self._where}.{key}[{i}]") for i, item in enumerate(items)]

    def raw(self, key: str) -> dict[str, Any]:
        return self.get(key, dict)

    def done(self) -> None:
        unknown = set(self._data) - self._used
        if unknown:
            raise ConfigError(f"{self._where}: unknown key(s) {', '.join(sorted(unknown))}")


def _parse(root: _Section) -> RobotConfig:
    direct = [_parse_joint(s) for s in root.sections("joint")]
    differential = [_parse_differential(s) for s in root.sections("differential")]
    drives = tuple(direct + differential)
    motors = {name: _parse_settings(values, f"motors.{name}") for name, values in root.raw("motors").items()}
    config = RobotConfig(
        firmware=root.get("firmware", str),
        control_rate_hz=_positive(root.get("control_rate_hz", float), "control_rate_hz"),
        bus=_parse_bus(root.section("bus")),
        telemetry_ms=_parse_telemetry(root.section("telemetry")),
        safety=_parse_safety(root.section("safety")),
        park=_parse_park(root.section("park")),
        drives=drives,
        gamepad=_parse_gamepad(root.section("gamepad")),
        common_settings=_parse_settings(root.raw("odrive"), "odrive"),
        motors=motors,
    )
    root.done()
    _check_consistency(config)
    return config


def _parse_bus(s: _Section) -> BusSpec:
    spec = BusSpec(s.get("interface", str), s.get("channel", str), s.get("bitrate", int))
    s.done()
    return spec


def _parse_telemetry(s: _Section) -> dict[str, int]:
    rates = {key: s.get(key, int) for key in _TELEMETRY_ENDPOINTS}
    s.done()
    for key in ("heartbeat_ms", "encoder_ms"):
        if rates[key] <= 0:
            raise ConfigError(f"telemetry.{key} must be on: the supervisor depends on it")
    return rates


def _parse_safety(s: _Section) -> SafetySpec:
    spec = SafetySpec(
        heartbeat_timeout_s=_positive(s.get("heartbeat_timeout_ms", float), "heartbeat_timeout_ms") / 1000,
        encoder_timeout_s=_positive(s.get("encoder_timeout_ms", float), "encoder_timeout_ms") / 1000,
        max_fet_temp_c=s.get("max_fet_temp_c", float),
        min_bus_voltage=s.get("min_bus_voltage", float),
        max_bus_voltage=s.get("max_bus_voltage", float),
        max_current_fraction=s.get("max_current_fraction", float),
        transient_s=_positive(s.get("transient_s", float), "transient_s"),
    )
    s.done()
    if spec.min_bus_voltage >= spec.max_bus_voltage:
        raise ConfigError("safety: min_bus_voltage must be below max_bus_voltage")
    if not 0.0 < spec.max_current_fraction <= 1.0:
        raise ConfigError("safety: max_current_fraction must be in (0, 1]")
    return spec


def _parse_park(s: _Section) -> ParkSpec:
    stages = tuple(tuple(_strings(stage, "park.stages")) for stage in s.get("stages", list))
    spec = ParkSpec(stages, _positive(s.get("stage_timeout_s", float), "park.stage_timeout_s"))
    s.done()
    return spec


def _parse_joint(s: _Section) -> DriveSpec:
    name = s.get("name", str)
    nodes = _node_ids(s.get("nodes", list), name)
    signs = tuple(s.get("signs", list, [1] * len(nodes)))
    unit, turns_per_unit = _scale(s, name)
    joint = _joint_limits(s, name, unit)
    drive = DriveSpec("direct", s.get("motor", str), nodes, signs, turns_per_unit, (joint,))
    s.done()
    _check_signs(drive)
    return drive


def _parse_differential(s: _Section) -> DriveSpec:
    nodes = _node_ids(s.get("nodes", list), "differential")
    if len(nodes) != 2:
        raise ConfigError("differential: needs exactly two nodes")
    ratio = _positive(s.get("gear_ratio", float), "differential.gear_ratio")
    bend = _joint_limits(s.section("bend"), None, "deg")
    twist = _joint_limits(s.section("twist"), None, "deg")
    drive = DriveSpec("differential", s.get("motor", str), nodes, (1, 1), ratio / _DEGREES_PER_TURN, (bend, twist))
    s.done()
    return drive


def _scale(s: _Section, name: str) -> tuple[Unit, float]:
    ratio = s.get("gear_ratio", float, 0.0)
    mm_per_turn = s.get("mm_per_turn", float, 0.0)
    if (ratio > 0) == (mm_per_turn > 0):
        raise ConfigError(f"joint '{name}': set exactly one of gear_ratio or mm_per_turn, positive")
    return ("deg", ratio / _DEGREES_PER_TURN) if ratio > 0 else ("mm", 1.0 / mm_per_turn)


def _joint_limits(s: _Section, name: str | None, unit: Unit) -> JointSpec:
    """Reads name (when not already known), limits, speed and tolerance, and closes the section."""
    joint = JointSpec(
        name=name if name is not None else s.get("name", str),
        unit=unit,
        min=s.get("min", float),
        max=s.get("max", float),
        max_speed=s.get("max_speed", float),
        tolerance=s.get("tolerance", float),
    )
    if name is None:
        s.done()
    if not (joint.min < joint.max and joint.min <= 0.0 <= joint.max):
        raise ConfigError(f"joint '{joint.name}': limits must satisfy min <= 0 (dock) <= max, min < max")
    _positive(joint.max_speed, f"{joint.name}.max_speed")
    _positive(joint.tolerance, f"{joint.name}.tolerance")
    return joint


def _parse_gamepad(s: _Section) -> GamepadSpec:
    bindings = tuple(_parse_binding(b) for b in s.sections("bindings"))
    spec = GamepadSpec(
        deadman_button=s.get("deadman_button", int),
        shift_button=s.get("shift_button", int),
        dead_zone=s.get("dead_zone", float),
        rate_hz=_positive(s.get("rate_hz", float), "gamepad.rate_hz"),
        bindings=bindings,
    )
    s.done()
    if not 0.0 <= spec.dead_zone < 1.0:
        raise ConfigError("gamepad.dead_zone must be in [0, 1)")
    return spec


def _parse_binding(s: _Section) -> GamepadBinding:
    when = s.get("when", str, "always")
    if when not in ("always", "shifted", "unshifted"):
        raise ConfigError(f"gamepad binding: when must be always, shifted or unshifted, got '{when}'")
    binding = GamepadBinding(
        axis=s.get("axis", int),
        joint=s.get("joint", str),
        speed=s.get("speed", float),
        when=cast(Shift, when),
        trigger=s.get("trigger", bool, False),
    )
    s.done()
    return binding


def _parse_settings(values: Mapping[str, Any], where: str) -> dict[str, SettingValue]:
    return {path: _setting_value(path, value, where) for path, value in values.items()}


def _setting_value(path: str, value: Any, where: str) -> SettingValue:
    if isinstance(value, str):
        enum_type = _ENUM_SETTINGS.get(path)
        if enum_type is None:
            raise ConfigError(f"{where}: '{path}' is not an enum setting and cannot take '{value}'")
        if value not in enum_type.__members__:
            raise ConfigError(f"{where}: '{path}' must be one of {', '.join(enum_type.__members__)}")
        return int(enum_type[value])
    if not isinstance(value, (bool, int, float)):
        raise ConfigError(f"{where}: '{path}' must be a number, bool or enum name")
    return value


def _check_consistency(config: RobotConfig) -> None:
    all_nodes = [node for drive in config.drives for node in drive.nodes]
    duplicates = {node for node in all_nodes if all_nodes.count(node) > 1}
    if duplicates:
        raise ConfigError(f"node(s) {sorted(duplicates)} drive more than one joint")

    names = [joint.name for drive in config.drives for joint in drive.joints]
    _require_unique(names, "joint names")

    for drive in config.drives:
        if drive.motor not in config.motors:
            raise ConfigError(f"joint '{drive.joints[0].name}': no [motors.{drive.motor}] profile")
        if DECEL_LIMIT_SETTING not in config.motors[drive.motor]:
            raise ConfigError(f"motors.{drive.motor}: '{DECEL_LIMIT_SETTING}' is required to plan stops")

    parked = [name for stage in config.park.stages for name in stage]
    _require_unique(parked, "park stages")
    _require_known(parked, names, "park stages")
    if set(parked) != set(names):
        raise ConfigError(f"park stages must include every joint; missing {sorted(set(names) - set(parked))}")

    _require_known([b.joint for b in config.gamepad.bindings], names, "gamepad bindings")


def _check_signs(drive: DriveSpec) -> None:
    if len(drive.signs) != len(drive.nodes) or any(sign not in (1, -1) for sign in drive.signs):
        raise ConfigError(f"joint '{drive.joints[0].name}': signs must be one +1/-1 per node")


def _node_ids(values: list[Any], where: str) -> tuple[int, ...]:
    if not values or not all(isinstance(n, int) and 0 <= n <= MAX_NODE_ID for n in values):
        raise ConfigError(f"{where}: nodes must be a non-empty list of ids in 0..{MAX_NODE_ID}")
    return tuple(values)


def _strings(values: Iterable[Any], where: str) -> list[str]:
    values = list(values)
    if not all(isinstance(v, str) for v in values):
        raise ConfigError(f"{where}: expected a list of joint names")
    return values


def _require_unique(names: list[str], where: str) -> None:
    repeated = sorted({name for name in names if names.count(name) > 1})
    if repeated:
        raise ConfigError(f"{where}: {', '.join(repeated)} listed more than once")


def _require_known(names: Iterable[str], known: Iterable[str], where: str) -> None:
    unknown = sorted(set(names) - set(known))
    if unknown:
        raise ConfigError(f"{where}: unknown joint(s) {', '.join(unknown)}")


def _positive(value: float, name: str) -> float:
    if value <= 0:
        raise ConfigError(f"{name} must be positive")
    return value
