import random

import pytest

from darm.arm import drives
from darm.config import RobotConfig


@pytest.fixture
def by_node(config: RobotConfig) -> dict[int, drives.Drive]:
    built = [drives.build(spec) for spec in config.drives]
    return {node: drive for drive in built for node in drive.nodes}


def test_direct_drive_scales_by_gear_ratio(by_node: dict[int, drives.Drive]) -> None:
    assert by_node[0].to_motor([40.0]) == pytest.approx((1.0,))
    assert by_node[0].to_joint([1.0]) == pytest.approx((40.0,))


def test_ganged_motors_turn_opposite_ways_for_one_joint(by_node: dict[int, drives.Drive]) -> None:
    shoulder = by_node[1]
    assert shoulder.to_motor([-40.0]) == pytest.approx((-1.0, 1.0))
    assert shoulder.to_joint([-1.0, 1.0]) == pytest.approx((-40.0,))
    assert shoulder.disagreement([-1.0, 1.0]) == pytest.approx(0.0)


def test_ganged_disagreement_is_measured_in_joint_units(by_node: dict[int, drives.Drive]) -> None:
    assert by_node[1].disagreement([-1.0, 1.1]) == pytest.approx(4.0)


def test_differential_twists_when_motors_agree_and_bends_when_they_oppose(by_node: dict[int, drives.Drive]) -> None:
    wrist = by_node[5]
    a, b = wrist.to_motor([0.0, 100.0])
    assert a == pytest.approx(b)
    a, b = wrist.to_motor([100.0, 0.0])
    assert a == pytest.approx(-b)


def test_every_transmission_inverts_exactly(by_node: dict[int, drives.Drive]) -> None:
    rng = random.Random(1)
    for drive in set(by_node.values()):
        for _ in range(100):
            joints = [rng.uniform(-300, 300) for _ in drive.joints]
            assert drive.to_joint(drive.to_motor(joints)) == pytest.approx(tuple(joints))


def test_mismatched_lengths_raise_instead_of_truncating(by_node: dict[int, drives.Drive]) -> None:
    with pytest.raises(ValueError, match="zip"):
        by_node[1].to_joint([1.0])
