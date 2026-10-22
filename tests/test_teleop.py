"""Every rule that decides whether a gamepad moves the arm, checked without a gamepad."""

import math

import pytest

from darm.config import GamepadBinding, GamepadSpec
from darm.teleop import apply_dead_zone, joint_velocities

DEADMAN = 4
SHIFT = 5
RELEASED_TRIGGER = -1.0

MAX_SPEEDS = {"base": 90.0, "shoulder": 60.0, "wrist_bend": 120.0, "elbow": 90.0, "gripper": 30.0}


def binding(axis: int, joint: str, speed: float, when: str = "always", trigger: bool = False) -> GamepadBinding:
    return GamepadBinding(axis=axis, joint=joint, speed=speed, when=when, trigger=trigger)  # type: ignore[arg-type]


SPEC = GamepadSpec(
    deadman_button=DEADMAN,
    shift_button=SHIFT,
    dead_zone=0.25,
    rate_hz=50.0,
    bindings=(
        binding(1, "base", 60.0, "unshifted"),
        binding(0, "shoulder", 60.0, "unshifted"),
        binding(0, "wrist_bend", 83.0, "shifted"),
        binding(3, "elbow", -60.0),
        binding(5, "gripper", -26.0, trigger=True),
        binding(2, "gripper", 26.0, trigger=True),
    ),
)


def axes(**deflections: float) -> list[float]:
    """Six centred axes with released triggers, overridden by keyword: a0=..., a5=..."""
    values = [0.0, 0.0, RELEASED_TRIGGER, 0.0, 0.0, RELEASED_TRIGGER]
    for name, value in deflections.items():
        values[int(name[1:])] = value
    return values


def buttons(*held: int) -> list[bool]:
    return [index in held for index in range(8)]


def velocities(axis_values: list[float], held: list[bool], spec: GamepadSpec = SPEC) -> dict[str, float]:
    return joint_velocities(axis_values, held, spec, MAX_SPEEDS)


# ------------------------------------------------------------------ dead-man


def test_nothing_moves_without_the_deadman() -> None:
    assert velocities(axes(a0=1.0, a1=1.0, a3=1.0, a5=1.0), buttons()) == {}


def test_nothing_moves_when_the_controller_has_no_deadman_button() -> None:
    assert velocities(axes(a1=1.0), [True] * 3) == {}


def test_centred_sticks_and_released_triggers_ask_for_nothing() -> None:
    assert velocities(axes(), buttons(DEADMAN)) == {}


# ------------------------------------------------------------------ dead zone


@pytest.mark.parametrize("deflection", [0.0, 0.1, 0.25, -0.25])
def test_inside_the_dead_zone_is_still(deflection: float) -> None:
    assert apply_dead_zone(deflection, 0.25) == 0.0


def test_dead_zone_output_rises_continuously_from_its_edge() -> None:
    assert apply_dead_zone(0.26, 0.25) == pytest.approx(0.01 / 0.75)
    assert apply_dead_zone(0.625, 0.25) == pytest.approx(0.5)
    assert apply_dead_zone(1.0, 0.25) == pytest.approx(1.0)


def test_dead_zone_is_symmetric_and_saturates() -> None:
    assert apply_dead_zone(-0.625, 0.25) == pytest.approx(-0.5)
    assert apply_dead_zone(1.5, 0.25) == pytest.approx(1.0)


def test_full_deflection_gives_the_bound_speed() -> None:
    assert velocities(axes(a1=1.0), buttons(DEADMAN)) == {"base": pytest.approx(60.0)}
    assert velocities(axes(a1=-1.0), buttons(DEADMAN)) == {"base": pytest.approx(-60.0)}


def test_negative_speed_inverts_an_axis() -> None:
    assert velocities(axes(a3=1.0), buttons(DEADMAN)) == {"elbow": pytest.approx(-60.0)}


# ------------------------------------------------------------------ shift


def test_unshifted_left_stick_drives_the_shoulder() -> None:
    assert velocities(axes(a0=1.0), buttons(DEADMAN)) == {"shoulder": pytest.approx(60.0)}


def test_shift_moves_the_left_stick_to_the_wrist() -> None:
    assert velocities(axes(a0=1.0), buttons(DEADMAN, SHIFT)) == {"wrist_bend": pytest.approx(83.0)}


def test_always_bindings_ignore_shift() -> None:
    assert velocities(axes(a3=1.0), buttons(DEADMAN, SHIFT)) == {"elbow": pytest.approx(-60.0)}


# ------------------------------------------------------------------ triggers


def test_a_released_trigger_does_nothing() -> None:
    assert "gripper" not in velocities(axes(), buttons(DEADMAN))


def test_a_full_trigger_gives_full_speed() -> None:
    assert velocities(axes(a2=1.0), buttons(DEADMAN)) == {"gripper": pytest.approx(26.0)}
    assert velocities(axes(a5=1.0), buttons(DEADMAN)) == {"gripper": pytest.approx(-26.0)}


def test_a_half_pulled_trigger_counts_from_its_rest_position() -> None:
    half_pulled = 0.0  # Halfway between rest (-1) and full (+1).
    expected = (0.5 - 0.25) / 0.75 * 26.0
    assert velocities(axes(a2=half_pulled), buttons(DEADMAN)) == {"gripper": pytest.approx(expected)}


def test_both_triggers_on_one_joint_cancel_out() -> None:
    assert velocities(axes(a2=1.0, a5=1.0), buttons(DEADMAN)) == {}


# ------------------------------------------------------------------ limits and bad input


def test_speed_is_capped_at_the_joints_max_speed() -> None:
    fast = GamepadSpec(DEADMAN, SHIFT, 0.25, 50.0, (binding(1, "base", 500.0),))
    assert velocities(axes(a1=1.0), buttons(DEADMAN), fast) == {"base": pytest.approx(MAX_SPEEDS["base"])}
    assert velocities(axes(a1=-1.0), buttons(DEADMAN), fast) == {"base": pytest.approx(-MAX_SPEEDS["base"])}


def test_joints_that_are_not_active_never_move() -> None:
    inactive = GamepadSpec(DEADMAN, SHIFT, 0.25, 50.0, (binding(1, "upper_arm", 60.0),))
    assert velocities(axes(a1=1.0), buttons(DEADMAN), inactive) == {}


def test_axes_the_controller_lacks_are_ignored() -> None:
    assert velocities([0.0, 1.0], buttons(DEADMAN)) == {"base": pytest.approx(60.0)}


def test_a_garbage_reading_counts_as_released() -> None:
    assert velocities(axes(a1=math.nan, a3=math.inf), buttons(DEADMAN)) == {}
