import random

import pytest

from darm.arm import drives
from darm.config import RobotConfig
from tests.conftest import node_of


@pytest.fixture
def by_joint(config: RobotConfig) -> dict[str, drives.Drive]:
    built = [drives.build(spec) for spec in config.drives]
    return {joint.name: drive for drive in built for joint in drive.joints}


def test_direct_drive_scales_by_gear_ratio(by_joint: dict[str, drives.Drive]) -> None:
    base = by_joint["base"]  # 27:1
    assert base.to_motor([40.0]) == pytest.approx((3.0,))
    assert base.to_joint([3.0]) == pytest.approx((40.0,))


def test_differential_twists_when_motors_agree_and_bends_when_they_oppose(by_joint: dict[str, drives.Drive]) -> None:
    wrist = by_joint["wrist_bend"]
    assert wrist.nodes == (node_of("wrist_bend"), node_of("wrist_bend") + 1)
    a, b = wrist.to_motor([0.0, 100.0])
    assert a == pytest.approx(b)
    a, b = wrist.to_motor([100.0, 0.0])
    assert a == pytest.approx(-b)


def test_every_transmission_inverts_exactly(by_joint: dict[str, drives.Drive]) -> None:
    rng = random.Random(1)
    for drive in set(by_joint.values()):
        for _ in range(100):
            joints = [rng.uniform(-300, 300) for _ in drive.joints]
            assert drive.to_joint(drive.to_motor(joints)) == pytest.approx(tuple(joints))


def test_the_wrong_number_of_motor_readings_raises(by_joint: dict[str, drives.Drive]) -> None:
    with pytest.raises(ValueError, match="unpack"):
        by_joint["base"].to_joint([1.0, 2.0])
    with pytest.raises(ValueError, match="unpack"):
        by_joint["wrist_twist"].to_joint([1.0])
