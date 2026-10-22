"""The arm against simulated ODrives: taking control, moving, every fault path, and parking."""

import re
import struct
import time
from collections.abc import Callable

import pytest

from darm.arm.arm import ArmState, MotionError, StartupError
from darm.odrive.protocol import ODriveError
from darm.sim import SimNetwork, place
from tests.conftest import ROBOT, Rig, eventually, motor_turns, node_of

RigFactory = Callable[..., Rig]


def at(**joints: float) -> dict[int, float]:
    """Motor positions that put direct-drive joints where named: at(elbow=-45.0)."""
    return {node_of(joint): motor_turns(joint, value) for joint, value in joints.items()}


# ------------------------------------------------------------------------------ taking control


def test_engaging_holds_every_joint_exactly_where_it_is(make_rig: RigFactory) -> None:
    rig = make_rig(lambda sim: place(sim, at(elbow=-45.0)))
    rig.arm.engage()
    assert rig.arm.state is ArmState.ARMED
    time.sleep(0.3)
    assert rig.arm.measured()["elbow"] == pytest.approx(-45.0)
    assert rig.arm.targets()["elbow"] == pytest.approx(-45.0)
    assert rig.journal.was_docked() is False


def test_missing_nodes_disable_only_their_joints(make_rig: RigFactory) -> None:
    rig = make_rig(node_ids=tuple(n for n in ROBOT.node_ids if n != node_of("gripper")))
    assert "gripper" not in rig.arm.joint_names
    rig.arm.engage()
    assert rig.arm.state is ArmState.ARMED


@pytest.mark.parametrize(
    ("prepare", "message"),
    [
        (lambda sim: setattr(sim.nodes[3], "firmware", (0, 6, 10)), "node 3 runs 0.6.10"),
        (lambda sim: sim.nodes[2].fault(ODriveError.DRV_FAULT), "node 2: DRV_FAULT"),
        (lambda sim: place(sim, at(elbow=30.0)), "elbow: at 30.0 deg"),
        (lambda sim: _set_float(sim, 0, "axis0.config.init_pos", 1.5), "init_pos is 1.5"),
        (lambda sim: _set_bool(sim, 6, "axis0.config.enable_watchdog", True), "watchdog is on"),
    ],
)
def test_engaging_refuses_anything_unsafe(
    make_rig: RigFactory, prepare: Callable[[SimNetwork], None], message: str
) -> None:
    rig = make_rig(prepare)
    with pytest.raises(StartupError, match=message):
        rig.arm.engage()
    assert rig.arm.state is ArmState.IDLE


def test_engaging_refuses_after_a_power_cycle_away_from_the_dock(make_rig: RigFactory) -> None:
    rig = make_rig()
    rig.journal.mark_away()
    with pytest.raises(StartupError, match="darm home"):
        rig.arm.engage()


def test_a_script_restart_mid_pose_is_not_mistaken_for_a_power_cycle(make_rig: RigFactory) -> None:
    rig = make_rig(lambda sim: place(sim, at(elbow=-45.0)))
    rig.journal.mark_away()
    rig.arm.engage()
    assert rig.arm.state is ArmState.ARMED


# ------------------------------------------------------------------------------ moving


def test_targets_are_clamped_to_limits(make_rig: RigFactory) -> None:
    rig = make_rig()
    rig.arm.engage()
    assert rig.arm.set_target("elbow", 50.0) == 0.0
    assert rig.arm.set_target("base", -999.0) == -192.0
    assert rig.arm.set_target("gripper", 100.0) == 25.7


def test_nudges_never_run_far_ahead_of_the_joint(make_rig: RigFactory) -> None:
    rig = make_rig()
    rig.arm.engage()
    for _ in range(50):
        rig.arm.nudge("base", 10.0)
    assert rig.arm.targets()["base"] <= ROBOT.joints["base"].max_speed * 0.25 + 1.0


def test_a_nudge_never_pulls_a_target_back_against_its_direction(make_rig: RigFactory) -> None:
    rig = make_rig()
    rig.arm.engage()
    rig.arm.set_target("base", 150.0)
    assert rig.arm.nudge("base", 1.0) == 150.0
    assert rig.arm.nudge("base", -1.0) < 150.0


def test_targets_are_ignored_unless_armed(make_rig: RigFactory) -> None:
    rig = make_rig()
    assert rig.arm.set_target("elbow", -30.0) == 0.0
    assert rig.arm.targets() == {}


def test_motion_is_slewed_at_the_joint_speed_limit(make_rig: RigFactory) -> None:
    rig = make_rig()
    rig.arm.engage()
    rig.arm.set_target("upper_arm", 180.0)
    time.sleep(0.5)
    position = rig.arm.measured()["upper_arm"]
    assert 0.0 < position <= ROBOT.joints["upper_arm"].max_speed * 0.5 + 5.0


def test_move_to_blocks_until_every_joint_has_settled(make_rig: RigFactory) -> None:
    rig = make_rig()
    rig.arm.engage()
    rig.arm.move_to({"elbow": -20.0, "wrist_twist": 15.0}, timeout=5.0)
    measured = rig.arm.measured()
    assert measured["elbow"] == pytest.approx(-20.0, abs=0.01)
    assert measured["wrist_twist"] == pytest.approx(15.0, abs=0.01)


# ------------------------------------------------------------------------------ faults


def test_a_silent_node_freezes_everything_else_where_it_stands(make_rig: RigFactory) -> None:
    rig = make_rig()
    rig.arm.engage()
    rig.arm.set_target("base", 90.0)
    time.sleep(0.3)
    with rig.sim.locked():
        rig.sim.nodes[node_of("gripper")].silent = True
    assert eventually(lambda: rig.arm.state is ArmState.FROZEN)
    assert f"node {node_of('gripper')}" in str(rig.arm.fault)
    stop = rig.arm.targets()["base"]
    time.sleep(1.0)
    assert rig.arm.measured()["base"] == pytest.approx(stop, abs=0.5)
    assert rig.arm.set_target("base", 0.0) == pytest.approx(stop)


def test_freezing_brakes_forward_instead_of_snapping_back(make_rig: RigFactory) -> None:
    rig = make_rig()
    rig.arm.engage()
    rig.arm.set_target("upper_arm", 180.0)
    time.sleep(0.6)
    at_freeze = rig.arm.measured()["upper_arm"]
    rig.arm.freeze("test")
    time.sleep(1.0)
    assert rig.arm.measured()["upper_arm"] >= at_freeze


def test_a_disarmed_odrive_freezes_the_arm_and_says_why(make_rig: RigFactory) -> None:
    rig = make_rig()
    rig.arm.engage()
    with rig.sim.locked():
        rig.sim.nodes[3].fault(ODriveError.DC_BUS_OVER_REGEN_CURRENT)
    assert eventually(lambda: rig.arm.state is ArmState.FROZEN)
    assert "DC_BUS_OVER_REGEN_CURRENT" in str(rig.arm.fault)


def test_a_blocked_joint_freezes_the_arm(make_rig: RigFactory) -> None:
    rig = make_rig()
    rig.arm.engage()
    with rig.sim.locked():
        rig.sim.nodes[node_of("elbow")].blocked = True
    rig.arm.set_target("elbow", -20.0)
    assert eventually(lambda: rig.arm.state is ArmState.FROZEN, timeout=3.0)
    assert "elbow: stuck" in str(rig.arm.fault)


def test_clearing_a_fault_rearms_and_holds_where_it_is(make_rig: RigFactory) -> None:
    rig = make_rig()
    rig.arm.engage()
    with rig.sim.locked():
        rig.sim.nodes[node_of("upper_arm")].fault(ODriveError.DRV_FAULT)
    assert eventually(lambda: rig.arm.state is ArmState.FROZEN)
    with rig.sim.locked():
        rig.sim.nodes[node_of("upper_arm")].teleport(motor_turns("upper_arm", -10.0))  # It sagged while disarmed.
    rig.arm.clear_fault()
    assert rig.arm.state is ArmState.ARMED
    assert rig.arm.targets()["upper_arm"] == pytest.approx(-10.0)


def test_a_fault_that_persists_cannot_be_cleared(make_rig: RigFactory) -> None:
    rig = make_rig()
    rig.arm.engage()
    with rig.sim.locked():
        rig.sim.nodes[0].fet_temp = 120.0
    assert eventually(lambda: rig.arm.state is ArmState.FROZEN)
    with pytest.raises(StartupError, match="FETs"):
        rig.arm.clear_fault()
    assert rig.arm.state is ArmState.FROZEN


# ------------------------------------------------------------------------------ parking


def test_parking_folds_stage_by_stage_then_releases_and_remembers(make_rig: RigFactory) -> None:
    rig = make_rig(lambda sim: place(sim, at(base=30.0, elbow=-60.0, gripper=10.0)))
    rig.arm.engage()
    stages: list[str] = []
    rig.arm.park(progress=stages.append)
    assert stages == [
        "parking gripper, wrist_bend, wrist_twist",
        "parking upper_arm, elbow",
        "parking base",
        "parking shoulder",
        "docked; releasing motors",
    ]
    assert rig.arm.state is ArmState.IDLE
    assert all(abs(v) < 0.5 for v in rig.arm.measured().values())
    assert rig.journal.was_docked() is True


def test_parking_stops_at_the_first_fault(make_rig: RigFactory) -> None:
    rig = make_rig(lambda sim: place(sim, at(elbow=-60.0)))
    rig.arm.engage()
    with rig.sim.locked():
        rig.sim.nodes[node_of("elbow")].blocked = True
    with pytest.raises(MotionError, match="frozen"):
        rig.arm.park()
    assert rig.arm.state is ArmState.FROZEN


def test_releasing_away_from_the_dock_is_refused(make_rig: RigFactory) -> None:
    rig = make_rig(lambda sim: place(sim, at(elbow=-60.0)))
    rig.arm.engage()
    with pytest.raises(MotionError, match=re.escape("not docked: elbow -60.0 deg")):
        rig.arm.release()
    assert rig.arm.state is ArmState.ARMED


# ------------------------------------------------------------------------------ helpers


def _set_float(sim: SimNetwork, node_id: int, path: str, value: float) -> None:
    sim.nodes[node_id].values[sim._endpoints[path].id] = struct.pack("<f", value)


def _set_bool(sim: SimNetwork, node_id: int, path: str, value: bool) -> None:
    sim.nodes[node_id].values[sim._endpoints[path].id] = struct.pack("<?", value)
