"""Console and operator-session logic, driven without a terminal or an arm."""

from __future__ import annotations

import threading
import time
from collections.abc import Callable
from pathlib import Path

import pytest

from darm import config as cfg
from darm.apps import tui
from darm.apps.console import JointKeys, Nudge
from darm.arm.arm import ArmState, Fault, MotionError
from darm.config import JointSpec

ROOT = Path(__file__).resolve().parents[1]
WAIT_S = 2.0


def joint(name: str, unit: str = "deg") -> JointSpec:
    return JointSpec(name=name, unit=unit, min=-90.0, max=90.0, max_speed=60.0, tolerance=1.0)  # type: ignore[arg-type]


# ------------------------------------------------------------------ joint keys


@pytest.fixture
def keys() -> JointKeys:
    return JointKeys([joint("base"), joint("shoulder"), joint("gripper", "mm")])


def test_first_joint_starts_selected(keys: JointKeys) -> None:
    assert keys.selected.name == "base"


def test_left_and_right_select_joints_and_stop_at_the_ends(keys: JointKeys) -> None:
    assert keys.handle("left") is None
    assert keys.selected.name == "base"
    keys.handle("right")
    keys.handle("right")
    keys.handle("right")
    assert keys.selected.name == "gripper"


@pytest.mark.parametrize(
    ("key", "delta"), [("up", 1.0), ("down", -1.0), ("page up", 10.0), ("page down", -10.0)]
)
def test_steps_in_degrees(keys: JointKeys, key: str, delta: float) -> None:
    assert keys.handle(key) == Nudge("base", delta)


@pytest.mark.parametrize(("key", "delta"), [("up", 0.5), ("page down", -5.0)])
def test_steps_in_millimetres(keys: JointKeys, key: str, delta: float) -> None:
    keys.handle("right")
    keys.handle("right")
    assert keys.handle(key) == Nudge("gripper", delta)


@pytest.mark.parametrize(
    ("key", "action"),
    [(" ", tui.Action.FREEZE), ("c", tui.Action.CLEAR), ("p", tui.Action.PARK), ("q", tui.Action.QUIT),
     ("esc", tui.Action.QUIT)],
)
def test_operator_keys(keys: JointKeys, key: str, action: tui.Action) -> None:
    assert keys.handle(key) is action


def test_unknown_keys_do_nothing(keys: JointKeys) -> None:
    assert keys.handle("x") is None
    assert keys.selected.name == "base"


def test_a_console_needs_at_least_one_joint() -> None:
    with pytest.raises(ValueError, match="no joints"):
        JointKeys([])


# ------------------------------------------------------------------ operator session


class FakeArm:
    """Records what the session asks of the arm. park() runs whatever the test plugs in."""

    def __init__(self, park: Callable[[], None] = lambda: None) -> None:
        self.state = ArmState.ARMED
        self.fault: Fault | None = None
        self.joint_names = ["base"]
        self.freezes: list[str] = []
        self.clears = 0
        self._park = park

    def freeze(self, reason: str) -> None:
        self.freezes.append(reason)
        self.state = ArmState.FROZEN
        self.fault = Fault((reason,), 0.0)

    def clear_fault(self) -> None:
        self.clears += 1

    def park(self, progress: Callable[[str], None]) -> None:
        progress("parking")
        self._park()


def session_for(arm: FakeArm) -> tui.OperatorSession:
    return tui.OperatorSession(arm)  # type: ignore[arg-type]


def wait_until_idle(session: tui.OperatorSession) -> None:
    deadline = time.monotonic() + WAIT_S
    while session.busy and time.monotonic() < deadline:
        time.sleep(0.005)
    assert not session.busy


def test_freeze_is_immediate() -> None:
    arm = FakeArm()
    assert session_for(arm).perform(tui.Action.FREEZE)
    assert arm.freezes == ["operator"]


def test_quit_ends_the_session_without_touching_the_arm() -> None:
    arm = FakeArm()
    assert not session_for(arm).perform(tui.Action.QUIT)
    assert arm.freezes == []


def test_clear_runs_in_the_background() -> None:
    arm = FakeArm()
    session = session_for(arm)
    session.perform(tui.Action.CLEAR)
    wait_until_idle(session)
    assert arm.clears == 1


def test_a_successful_park_marks_the_session_parked() -> None:
    session = session_for(FakeArm())
    session.perform(tui.Action.PARK)
    wait_until_idle(session)
    assert session.parked
    assert session.activity == ""


def test_a_failed_park_is_reported_not_raised() -> None:
    def refuse() -> None:
        raise MotionError("cannot move while frozen")

    arm = FakeArm(refuse)
    session = session_for(arm)
    session.perform(tui.Action.PARK)
    wait_until_idle(session)
    assert not session.parked
    assert arm.freezes == []


def test_a_crashing_park_freezes_the_arm() -> None:
    def crash() -> None:
        raise RuntimeError("bug")

    arm = FakeArm(crash)
    session = session_for(arm)
    session.perform(tui.Action.PARK)
    wait_until_idle(session)
    assert arm.freezes == ["parking crashed"]


def test_one_background_job_at_a_time() -> None:
    release = threading.Event()
    arm = FakeArm(lambda: release.wait(WAIT_S) and None)
    session = session_for(arm)
    session.perform(tui.Action.PARK)
    session.perform(tui.Action.CLEAR)
    release.set()
    wait_until_idle(session)
    assert arm.clears == 0


def test_quitting_mid_park_freezes_the_arm_where_it_is() -> None:
    release = threading.Event()
    arm = FakeArm(lambda: release.wait(WAIT_S) and None)
    session = session_for(arm)
    session.perform(tui.Action.PARK)
    original_freeze = arm.freeze

    def freeze_and_release(reason: str) -> None:
        original_freeze(reason)
        release.set()  # A real park notices the freeze and stops.

    arm.freeze = freeze_and_release  # type: ignore[method-assign]
    session.shutdown()
    assert arm.freezes == ["operator quit while parking"]
    assert not session.busy


# ------------------------------------------------------------------ what the screen says


def test_status_line_names_the_fault() -> None:
    arm = FakeArm()
    arm.freeze("node 3: silent")
    attr, text = tui.status_line(arm, None)  # type: ignore[arg-type]
    assert attr == "fault"
    assert "node 3: silent" in text


def test_status_line_when_armed_and_idle() -> None:
    arm = FakeArm()
    assert tui.status_line(arm, None)[0] == "armed"  # type: ignore[arg-type]
    arm.state = ArmState.IDLE
    assert tui.status_line(arm, None)[0] == "idle"  # type: ignore[arg-type]


def test_summary_says_how_the_arm_was_left() -> None:
    arm = FakeArm()
    assert "holding" in tui.summary(arm)  # type: ignore[arg-type]
    arm.freeze("operator")
    assert "frozen" in tui.summary(arm)  # type: ignore[arg-type]


def test_node_labels_name_each_motor_of_a_joint() -> None:
    labels = tui.node_labels(cfg.load(ROOT / "darm.toml").drives)
    assert labels[0] == "base"
    assert (labels[1], labels[2]) == ("shoulder A", "shoulder B")
    assert (labels[5], labels[6]) == ("wrist A", "wrist B")
    assert labels[7] == "gripper"
