"""darm home: declare the arm's current pose to be the dock, zero for every joint.

ODrives count position from wherever they powered up, and darm treats zero as
the dock. When those two drift apart, because the arm was powered up somewhere
else or moved while unpowered, every limit is off by the same amount. With the
arm physically seated in the dock, this makes the ODrives agree again.

Only for an arm that is idle: re-zeroing a motor that is holding position would
make it chase the new zero.
"""

from __future__ import annotations

import argparse
import logging
import time
from collections.abc import Mapping
from typing import TYPE_CHECKING

from darm.apps._commissioning import busy_problems, confirm, present_nodes
from darm.arm import drives
from darm.config import RobotConfig
from darm.odrive.node import ODriveNode

if TYPE_CHECKING:
    from darm.cli import Context

log = logging.getLogger(__name__)

_SETTLE_S = 1.0


def register(commands: argparse._SubParsersAction[argparse.ArgumentParser]) -> None:
    parser = commands.add_parser(
        "home",
        help="make the current pose the docked zero (arm must be seated in the dock)",
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--yes", action="store_true", help="do not ask whether the arm is docked")
    parser.set_defaults(run=run)


def run(args: argparse.Namespace, ctx: Context) -> int:
    config = ctx.config
    if not confirm("Is the arm seated in the dock, every joint in its docked position?", args.yes):
        print("Not homed.")
        return 1
    with ctx.bus(config.node_ids) as bus:
        nodes = present_nodes(bus, config.node_ids)
        if not nodes:
            log.error("no ODrives answered")
            return 1
        problems = busy_problems(nodes.values())
        if problems:
            for problem in problems:
                log.error("%s", problem)
            return 1
        zeroed_at = time.monotonic()
        for node in nodes.values():
            node.set_absolute_position(0.0)
        off = _off_zero(config, nodes, zeroed_at)
    if off:
        log.error("still not at zero after homing: %s", ", ".join(off))
        return 1
    ctx.journal().mark_docked()
    missing = sorted(set(config.node_ids) - set(nodes))
    if missing:
        log.warning("node(s) %s were silent and are not homed", missing)
    print(f"Homed node(s) {', '.join(str(n) for n in nodes)}: the dock is zero.")
    return 0


def _off_zero(config: RobotConfig, nodes: Mapping[int, ODriveNode], since: float) -> list[str]:
    """Joints that, by fresh encoder frames, are not within tolerance of zero. Checked per joint, in joint units."""
    for node in nodes.values():
        if not node.wait_until(lambda n: n.encoder is not None and n.encoder.at > since, _SETTLE_S):
            return [f"node {node.node_id} sent no position"]
    turns = {node_id: node.encoder.value.pos for node_id, node in nodes.items() if node.encoder}
    off = []
    for drive in (drives.build(spec) for spec in config.drives if set(spec.nodes) <= set(nodes)):
        measured = drive.to_joint([turns[n] for n in drive.nodes])
        for joint, value in zip(drive.joints, measured, strict=True):
            if abs(value) > joint.tolerance:
                off.append(f"{joint.name} at {value:.2f} {joint.unit}")
    return off
