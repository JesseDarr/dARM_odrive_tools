"""ODrive CANSimple wire protocol: message ids, enums, and payload codecs.

Pure functions and immutable values only. No I/O, no threads, no state.
Every byte layout the project depends on lives in this file and nowhere else.

Reference: https://docs.odriverobotics.com/v/latest/manual/can-protocol.html
"""

import enum
import struct
from dataclasses import dataclass

CMD_BITS = 5
CMD_MASK = (1 << CMD_BITS) - 1
BROADCAST_NODE_ID = 0x3F  # Reaches every ODrive. Also the id of an ODrive that has none yet (unaddressed).
MAX_NODE_ID = BROADCAST_NODE_ID - 1
NODE_FILTER_MASK = BROADCAST_NODE_ID << CMD_BITS
SERIAL_NUMBER_BYTES = 6


class Cmd(enum.IntEnum):
    GET_VERSION = 0x00
    HEARTBEAT = 0x01
    ESTOP = 0x02
    GET_ERROR = 0x03
    RX_SDO = 0x04
    TX_SDO = 0x05
    ADDRESS = 0x06
    SET_AXIS_STATE = 0x07
    GET_ENCODER_ESTIMATES = 0x09
    SET_CONTROLLER_MODE = 0x0B
    SET_INPUT_POS = 0x0C
    SET_INPUT_VEL = 0x0D
    SET_INPUT_TORQUE = 0x0E
    SET_LIMITS = 0x0F
    SET_TRAJ_VEL_LIMIT = 0x11
    SET_TRAJ_ACCEL_LIMITS = 0x12
    GET_IQ = 0x14
    GET_TEMPERATURE = 0x15
    REBOOT = 0x16
    GET_BUS_VOLTAGE_CURRENT = 0x17
    CLEAR_ERRORS = 0x18
    SET_ABSOLUTE_POSITION = 0x19
    GET_TORQUES = 0x1C
    GET_POWERS = 0x1D


def arbitration_id(node_id: int, cmd: int) -> int:
    if not 0 <= node_id <= BROADCAST_NODE_ID:
        raise ValueError(f"node id {node_id} outside 0..{BROADCAST_NODE_ID}")
    return (node_id << CMD_BITS) | cmd


def base_version(firmware: str) -> str:
    """'0.6.12-1' -> '0.6.12'. Hotfix suffixes never reach the wire: Get_Version reports only major.minor.revision."""
    return firmware.partition("-")[0]


def split_arbitration_id(arb_id: int) -> tuple[int, int]:
    return arb_id >> CMD_BITS, arb_id & CMD_MASK


class _TolerantIntEnum(enum.IntEnum):
    """Decodes values a newer firmware may add instead of raising.

    A ValueError on the bus reader thread would silently stop all telemetry,
    which the supervisor would then report as a dead node. An UNKNOWN_n member
    keeps the frame flowing and keeps the real number visible.
    """

    @classmethod
    def _missing_(cls, value: object) -> _TolerantIntEnum | None:
        if not isinstance(value, int):
            return None
        member = int.__new__(cls, value)
        member._name_ = f"UNKNOWN_{value}"
        member._value_ = value
        return member


class AxisState(_TolerantIntEnum):
    UNDEFINED = 0
    IDLE = 1
    STARTUP_SEQUENCE = 2
    FULL_CALIBRATION_SEQUENCE = 3
    MOTOR_CALIBRATION = 4
    ENCODER_INDEX_SEARCH = 6
    ENCODER_OFFSET_CALIBRATION = 7
    CLOSED_LOOP_CONTROL = 8
    LOCKIN_SPIN = 9
    ENCODER_DIR_FIND = 10
    HOMING = 11
    ENCODER_HALL_POLARITY_CALIBRATION = 12
    ENCODER_HALL_PHASE_CALIBRATION = 13
    ANTICOGGING_CALIBRATION = 14
    HARMONIC_CALIBRATION = 15
    HARMONIC_CALIBRATION_COMMUTATION = 16


class ControlMode(_TolerantIntEnum):
    VOLTAGE_CONTROL = 0
    TORQUE_CONTROL = 1
    VELOCITY_CONTROL = 2
    POSITION_CONTROL = 3


class InputMode(_TolerantIntEnum):
    INACTIVE = 0
    PASSTHROUGH = 1
    VEL_RAMP = 2
    POS_FILTER = 3
    MIX_CHANNELS = 4
    TRAP_TRAJ = 5
    TORQUE_RAMP = 6
    MIRROR = 7
    TUNING = 8


class MotorType(_TolerantIntEnum):
    PMSM_CURRENT_CONTROL = 0
    PMSM_VOLTAGE_CONTROL = 2  # Gimbal motors: no current sensing, V = I * phase_resistance.
    ACIM = 3
    HIGH_CURRENT = 0  # Legacy names, still accepted by the firmware.
    GIMBAL = 2


class ProcedureResult(_TolerantIntEnum):
    SUCCESS = 0
    BUSY = 1
    CANCELLED = 2
    DISARMED = 3
    NO_RESPONSE = 4
    POLE_PAIR_CPR_MISMATCH = 5
    PHASE_RESISTANCE_OUT_OF_RANGE = 6
    PHASE_INDUCTANCE_OUT_OF_RANGE = 7
    UNBALANCED_PHASES = 8
    INVALID_MOTOR_TYPE = 9
    ILLEGAL_HALL_STATE = 10
    TIMEOUT = 11
    HOMING_WITHOUT_ENDSTOP = 12
    INVALID_STATE = 13
    NOT_CALIBRATED = 14
    NOT_CONVERGING = 15
    REQUESTED_CURRENT_TOO_HIGH = 16


class RebootAction(_TolerantIntEnum):
    REBOOT = 0
    SAVE_CONFIGURATION = 1
    ERASE_CONFIGURATION = 2


class ODriveError(enum.IntFlag):
    NONE = 0
    INITIALIZING = 0x1
    SYSTEM_LEVEL = 0x2
    TIMING_ERROR = 0x4
    MISSING_ESTIMATE = 0x8
    BAD_CONFIG = 0x10
    DRV_FAULT = 0x20
    MISSING_INPUT = 0x40
    DC_BUS_OVER_VOLTAGE = 0x100
    DC_BUS_UNDER_VOLTAGE = 0x200
    DC_BUS_OVER_CURRENT = 0x400
    DC_BUS_OVER_REGEN_CURRENT = 0x800
    CURRENT_LIMIT_VIOLATION = 0x1000
    MOTOR_OVER_TEMP = 0x2000
    INVERTER_OVER_TEMP = 0x4000
    VELOCITY_LIMIT_VIOLATION = 0x8000
    POSITION_LIMIT_VIOLATION = 0x10000
    REQUESTED_CURRENT_TOO_HIGH = 0x20000
    WATCHDOG_TIMER_EXPIRED = 0x1000000
    ESTOP_REQUESTED = 0x2000000
    SPINOUT_DETECTED = 0x4000000
    BRAKE_RESISTOR_DISARMED = 0x8000000
    THERMISTOR_DISCONNECTED = 0x10000000
    CALIBRATION_ERROR = 0x40000000


def describe_errors(errors: int) -> str:
    """'DC_BUS_UNDER_VOLTAGE | SPINOUT_DETECTED', or 'none'. Unknown bits stay visible as hex."""
    if errors == 0:
        return "none"
    names = [flag.name for flag in ODriveError if flag and flag.name and errors & flag]
    unknown = errors & ~sum(flag.value for flag in ODriveError)
    if unknown:
        names.append(f"0x{unknown:08X}")
    return " | ".join(names)


# ---------------------------------------------------------------- telemetry (ODrive -> host)


@dataclass(frozen=True, slots=True)
class Heartbeat:
    axis_error: int
    axis_state: AxisState
    procedure_result: ProcedureResult
    trajectory_done: bool


@dataclass(frozen=True, slots=True)
class Errors:
    active: int
    disarm_reason: int


@dataclass(frozen=True, slots=True)
class EncoderEstimates:
    pos: float  # motor turns
    vel: float  # motor turns/s


@dataclass(frozen=True, slots=True)
class Iq:
    setpoint: float  # A
    measured: float  # A


@dataclass(frozen=True, slots=True)
class Temperature:
    fet: float  # degC
    motor: float  # degC, NaN when no motor thermistor is fitted


@dataclass(frozen=True, slots=True)
class BusVoltageCurrent:
    voltage: float  # V
    current: float  # A


@dataclass(frozen=True, slots=True)
class Torques:
    target: float  # Nm
    estimate: float  # Nm


@dataclass(frozen=True, slots=True)
class Powers:
    electrical: float  # W
    mechanical: float  # W


@dataclass(frozen=True, slots=True)
class AddressReply:
    node_id: int  # BROADCAST_NODE_ID when unaddressed
    serial_number: int

    @property
    def serial(self) -> str:
        return f"{self.serial_number:012X}"


@dataclass(frozen=True, slots=True)
class Version:
    protocol: int
    hw_product_line: int
    hw_version: int
    hw_variant: int
    fw_major: int
    fw_minor: int
    fw_revision: int
    fw_unreleased: int

    @property
    def firmware(self) -> str:
        return f"{self.fw_major}.{self.fw_minor}.{self.fw_revision}"

    @property
    def hardware(self) -> str:
        return f"{self.hw_product_line}.{self.hw_version}.{self.hw_variant}"


_HEARTBEAT = struct.Struct("<IBBB")
_TWO_U32 = struct.Struct("<II")
_TWO_F32 = struct.Struct("<ff")
_VERSION = struct.Struct("<8B")
_U32 = struct.Struct("<I")
_U8 = struct.Struct("<B")
_F32 = struct.Struct("<f")
_INPUT_POS = struct.Struct("<fhh")
_SDO_HEADER = struct.Struct("<BHB")

_SDO_READ = 0
_SDO_WRITE = 1
_INT16_MAX = 0x7FFF
_FEEDFORWARD_SCALE = 1000  # Set_Input_Pos packs vel_ff and torque_ff as int16 in units of 0.001


def decode_heartbeat(data: bytes) -> Heartbeat:
    error, state, result, flags = _HEARTBEAT.unpack_from(data)
    return Heartbeat(error, AxisState(state), ProcedureResult(result), bool(flags & 0x01))


def decode_errors(data: bytes) -> Errors:
    return Errors(*_TWO_U32.unpack_from(data))


def decode_encoder_estimates(data: bytes) -> EncoderEstimates:
    return EncoderEstimates(*_TWO_F32.unpack_from(data))


def decode_iq(data: bytes) -> Iq:
    return Iq(*_TWO_F32.unpack_from(data))


def decode_temperature(data: bytes) -> Temperature:
    return Temperature(*_TWO_F32.unpack_from(data))


def decode_bus_voltage_current(data: bytes) -> BusVoltageCurrent:
    return BusVoltageCurrent(*_TWO_F32.unpack_from(data))


def decode_torques(data: bytes) -> Torques:
    return Torques(*_TWO_F32.unpack_from(data))


def decode_powers(data: bytes) -> Powers:
    return Powers(*_TWO_F32.unpack_from(data))


def decode_version(data: bytes) -> Version:
    return Version(*_VERSION.unpack_from(data))


def decode_address(data: bytes) -> AddressReply:
    end = 1 + SERIAL_NUMBER_BYTES
    if len(data) < end:
        raise struct.error(f"Address reply needs {end} bytes, got {len(data)}")
    return AddressReply(data[0], int.from_bytes(data[1:end], "little"))


# ---------------------------------------------------------------- commands (host -> ODrive)


def encode_set_axis_state(state: AxisState) -> bytes:
    return _U32.pack(state)


def encode_set_controller_mode(control: ControlMode, inp: InputMode) -> bytes:
    return _TWO_U32.pack(control, inp)


def encode_set_input_pos(pos: float, vel_ff: float = 0.0, torque_ff: float = 0.0) -> bytes:
    return _INPUT_POS.pack(pos, _to_int16(vel_ff * _FEEDFORWARD_SCALE), _to_int16(torque_ff * _FEEDFORWARD_SCALE))


def encode_set_input_vel(vel: float, torque_ff: float = 0.0) -> bytes:
    return _TWO_F32.pack(vel, torque_ff)


def encode_set_limits(vel_limit: float, current_limit: float) -> bytes:
    return _TWO_F32.pack(vel_limit, current_limit)


def encode_reboot(action: RebootAction) -> bytes:
    return _U8.pack(action)


def encode_clear_errors(identify: bool = False) -> bytes:
    """Clears errors. identify=True also blinks the LED until a later clear with identify=False."""
    return _U8.pack(identify)


def encode_address(node_id: int, serial_number: int) -> bytes:
    """Give node_id to the ODrive with this serial number. Serial 0 matches ANY ODrive that hears it."""
    return bytes([node_id]) + serial_number.to_bytes(SERIAL_NUMBER_BYTES, "little")


def encode_set_absolute_position(pos: float) -> bytes:
    return _F32.pack(pos)


def _to_int16(value: float) -> int:
    return max(-_INT16_MAX, min(_INT16_MAX, round(value)))


# ---------------------------------------------------------------- SDO (endpoint access)


def encode_sdo_read(endpoint_id: int) -> bytes:
    return _SDO_HEADER.pack(_SDO_READ, endpoint_id, 0)


def encode_sdo_write(endpoint_id: int, value: bytes = b"") -> bytes:
    frame = _SDO_HEADER.pack(_SDO_WRITE, endpoint_id, 0) + value
    if len(frame) > 8:
        raise ValueError(f"endpoint {endpoint_id}: {len(value)}-byte value does not fit in a classic CAN frame")
    return frame


def decode_sdo_response(data: bytes) -> tuple[int, bytes]:
    """TxSdo -> (endpoint_id, raw value bytes). The endpoint id is how replies are matched to requests."""
    _, endpoint_id, _ = _SDO_HEADER.unpack_from(data)
    return endpoint_id, bytes(data[_SDO_HEADER.size :])
