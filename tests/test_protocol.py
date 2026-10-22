import struct

import pytest

from darm.odrive import protocol as p


def test_arbitration_id_round_trips() -> None:
    for node_id in (0, 7, p.MAX_NODE_ID, p.BROADCAST_NODE_ID):
        for cmd in p.Cmd:
            assert p.split_arbitration_id(p.arbitration_id(node_id, cmd)) == (node_id, cmd)


def test_arbitration_id_rejects_node_ids_that_do_not_fit() -> None:
    with pytest.raises(ValueError, match="node id"):
        p.arbitration_id(p.BROADCAST_NODE_ID + 1, p.Cmd.HEARTBEAT)


def test_heartbeat_decodes_every_field() -> None:
    data = struct.pack("<IBBB", p.ODriveError.DRV_FAULT, 8, 0, 1) + b"\x00"
    heartbeat = p.decode_heartbeat(data)
    assert heartbeat.axis_error == p.ODriveError.DRV_FAULT
    assert heartbeat.axis_state is p.AxisState.CLOSED_LOOP_CONTROL
    assert heartbeat.procedure_result is p.ProcedureResult.SUCCESS
    assert heartbeat.trajectory_done


def test_unknown_enum_values_from_newer_firmware_decode_instead_of_raising() -> None:
    heartbeat = p.decode_heartbeat(struct.pack("<IBBB", 0, 99, 77, 0))
    assert heartbeat.axis_state == 99
    assert heartbeat.axis_state.name == "UNKNOWN_99"
    assert heartbeat.procedure_result.name == "UNKNOWN_77"


def test_set_input_pos_scales_and_saturates_feedforward() -> None:
    pos, vel_ff, torque_ff = struct.unpack("<fhh", p.encode_set_input_pos(1.5, vel_ff=2.0, torque_ff=1e6))
    assert pos == 1.5
    assert vel_ff == 2000
    assert torque_ff == 0x7FFF


def test_sdo_write_refuses_values_that_do_not_fit_a_classic_frame() -> None:
    with pytest.raises(ValueError, match="does not fit"):
        p.encode_sdo_write(1, b"\x00" * 8)


def test_sdo_response_reports_endpoint_and_value_and_ignores_reserved_bytes() -> None:
    endpoint_id, value = p.decode_sdo_response(bytes([0xAA, 0x1A, 0x01, 0xBB]) + struct.pack("<f", 2.5))
    assert endpoint_id == 0x011A
    assert struct.unpack("<f", value)[0] == 2.5


def test_describe_errors_names_known_bits_and_keeps_unknown_ones_visible() -> None:
    assert p.describe_errors(0) == "none"
    assert p.describe_errors(p.ODriveError.DC_BUS_UNDER_VOLTAGE | p.ODriveError.SPINOUT_DETECTED) == (
        "DC_BUS_UNDER_VOLTAGE | SPINOUT_DETECTED"
    )
    assert p.describe_errors(0x80) == "0x00000080"


def test_address_round_trips_and_rejects_short_replies() -> None:
    frame = p.encode_address(5, 0x2050_3333_4444)
    reply = p.decode_address(frame)
    assert (reply.node_id, reply.serial) == (5, "205033334444")
    with pytest.raises(struct.error):
        p.decode_address(frame[:3])


def test_base_version_drops_hotfix_suffix() -> None:
    assert p.base_version("0.6.12-1") == "0.6.12"
    assert p.base_version("0.6.12") == "0.6.12"


def test_legacy_motor_type_names_alias_the_current_ones() -> None:
    assert p.MotorType.GIMBAL is p.MotorType.PMSM_VOLTAGE_CONTROL
    assert p.MotorType.HIGH_CURRENT is p.MotorType.PMSM_CURRENT_CONTROL
