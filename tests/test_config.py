import math
from pathlib import Path

import pytest

from darm import config as cfg
from darm.odrive.protocol import ControlMode, InputMode


def load_variant(tmp_path: Path, text: str, old: str, new: str) -> cfg.RobotConfig:
    assert old in text, f"test is stale: {old!r} no longer in darm.toml"
    path = tmp_path / "variant.toml"
    path.write_text(text.replace(old, new, 1), encoding="utf-8")
    return cfg.load(path)


def test_darm_describes_every_joint_in_output_units(config: cfg.RobotConfig) -> None:
    joints = config.joints
    assert list(joints) == ["base", "shoulder", "upper_arm", "elbow", "gripper", "wrist_bend", "wrist_twist"]
    assert joints["base"].unit == "deg"
    assert joints["gripper"].unit == "mm"
    assert config.node_ids == tuple(range(7))


def test_gear_ratios_become_motor_turns_per_unit(config: cfg.RobotConfig) -> None:
    base = config.drive_for_node(0)
    assert base.turns_per_unit == pytest.approx(27.0 / 360.0)
    wrist = config.drive_for_node(5)
    assert wrist.kind == "differential"
    assert wrist.turns_per_unit == pytest.approx(12.96 / 360.0)
    assert config.drive_for_node(6).turns_per_unit == pytest.approx(1 / 35.186)


def test_settings_combine_common_telemetry_and_motor_profile(config: cfg.RobotConfig) -> None:
    settings = config.settings_for(3)
    assert settings["axis0.config.enable_watchdog"] is False
    assert math.isnan(settings["axis0.config.init_pos"])
    assert settings["axis0.config.can.heartbeat_msg_rate_ms"] == 50
    assert settings["axis0.controller.config.control_mode"] == ControlMode.POSITION_CONTROL
    assert settings["axis0.controller.config.input_mode"] == InputMode.TRAP_TRAJ
    assert settings["axis0.config.motor.pole_pairs"] == 20


def test_current_limits_apply_only_to_current_controlled_motors(config: cfg.RobotConfig) -> None:
    assert config.current_limit(0) == 40.0
    assert config.current_limit(5) is None
    assert config.current_limit(6) is None


def test_firmware_base_is_what_the_odrives_report(config: cfg.RobotConfig) -> None:
    assert config.firmware_base == "0.6.12"


@pytest.mark.parametrize(
    ("old", "new", "message"),
    [
        (
            'max_speed = 60.0\ntolerance = 3.0\n\n[[joint]]\nname = "shoulder"',
            'max_sped = 60.0\ntolerance = 3.0\n\n[[joint]]\nname = "shoulder"',
            "missing 'max_speed'",
        ),
        ("tolerance = 1.0", 'tolerance = 1.0\ncolour = "orange"', "unknown key.*colour"),
        ("node = 3\n", "node = 2\n", "drive more than one joint"),
        ("node = 1\n", "node = 1\nnodes = [1, 7]\n", "unknown key.*nodes"),
        ("min = -236.0", "min = 10.0", "min <= 0"),
        ("gear_ratio = 27.0\nmin = -192.0", "gear_ratio = 27.0\nmm_per_turn = 1.0\nmin = -192.0", "exactly one"),
        ('= "TRAP_TRAJ"', '= "TRAPEZOID"', "must be one of"),
        ('"axis0.config.enable_watchdog" = false', '"axis0.current_state" = 1', "read-only"),
        ('["base"],', "", "must include every joint"),
        ('joint = "elbow"', 'joint = "knee"', "unknown joint"),
        ("min_bus_voltage = 42.0", "min_bus_voltage = 60.0", "below max_bus_voltage"),
        ('"axis0.config.enable_watchdog"', '"axis0.config.enable_watchdgo"', "Did you mean"),
    ],
)
def test_config_mistakes_are_rejected_before_anything_moves(
    tmp_path: Path, config_text: str, old: str, new: str, message: str
) -> None:
    with pytest.raises((cfg.ConfigError, KeyError), match=message):
        load_variant(tmp_path, config_text, old, new)


def test_node_labels_name_each_motor_of_a_joint(config: cfg.RobotConfig) -> None:
    labels = config.node_labels()
    assert (labels[0], labels[1], labels[3]) == ("base", "shoulder", "elbow")
    assert (labels[4], labels[5]) == ("wrist A", "wrist B")
    assert labels[6] == "gripper"
