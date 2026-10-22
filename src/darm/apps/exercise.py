"""darm exercise: sweep one joint end to end a few times, then put it back where it was.

Fresh grease only spreads when the gears turn through their whole range. This
does that through the arm, so every limit, slew rate and safety check applies.
There is no continuous rotation, nothing bypasses the supervisor, and a fault
freezes the arm like any other motion. The rest of the arm holds still.
"""

import argparse
import logging
from typing import TYPE_CHECKING

from darm.apps._commissioning import confirm
from darm.arm.arm import Arm
from darm.config import ConfigError, JointSpec

if TYPE_CHECKING:
    from darm.cli import Context

log = logging.getLogger(__name__)

_TIMEOUT_MARGIN = 1.5  # On top of distance / max_speed, for acceleration and settling.
_TIMEOUT_SLACK_S = 3.0


def register(commands: argparse._SubParsersAction[argparse.ArgumentParser]) -> None:
    parser = commands.add_parser(
        "exercise",
        help="sweep a joint through its range to spread grease",
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("joint", help="joint name, as in darm.toml")
    parser.add_argument("--cycles", type=_positive_int, default=3, help="end-to-end sweeps (default 3)")
    parser.add_argument("--span", type=_fraction, default=0.9, help="share of the joint's range to cover (default 0.9)")
    parser.add_argument("--yes", action="store_true", help="do not ask before moving")
    parser.set_defaults(run=run)


def run(args: argparse.Namespace, ctx: Context) -> int:
    joint = ctx.config.joints.get(args.joint)
    if joint is None:
        raise ConfigError(f"no joint '{args.joint}'; joints: {', '.join(ctx.config.joints)}")
    low, high = sweep_range(joint, args.span)
    question = f"Sweep {joint.name} between {low:.1f} and {high:.1f} {joint.unit}, {args.cycles} time(s)? Keep clear."
    if not confirm(question, args.yes):
        print("Nothing moved.")
        return 1
    with ctx.arm() as arm:
        if joint.name not in arm.joint_names:
            log.error("%s is not available: its ODrives did not answer", joint.name)
            return 1
        arm.engage()
        try:
            _sweep(arm, joint, low, high, args.cycles)
        except KeyboardInterrupt:
            arm.freeze("exercise interrupted")
            raise
    return 0


def sweep_range(joint: JointSpec, span: float) -> tuple[float, float]:
    """The middle `span` of the joint's range: clear of both limits by the same margin."""
    middle = (joint.min + joint.max) / 2
    half = (joint.max - joint.min) / 2 * span
    return middle - half, middle + half


def _sweep(arm: Arm, joint: JointSpec, low: float, high: float, cycles: int) -> None:
    start = arm.targets()[joint.name]
    position = start
    for cycle in range(1, cycles + 1):
        log.info("%s: sweep %d of %d", joint.name, cycle, cycles)
        for target in (high, low):
            _move(arm, joint, position, target)
            position = target
    log.info("%s: returning to %.1f %s", joint.name, start, joint.unit)
    _move(arm, joint, position, start)


def _move(arm: Arm, joint: JointSpec, start: float, target: float) -> None:
    timeout = abs(target - start) / joint.max_speed * _TIMEOUT_MARGIN + _TIMEOUT_SLACK_S
    arm.move_to({joint.name: target}, timeout)


def _positive_int(text: str) -> int:
    value = int(text)
    if value < 1:
        raise argparse.ArgumentTypeError("must be at least 1")
    return value


def _fraction(text: str) -> float:
    value = float(text)
    if not 0.0 < value <= 1.0:
        raise argparse.ArgumentTypeError("must be in (0, 1]")
    return value
