"""darm errors: what each ODrive is complaining about, and optionally clear it.

Active errors are conditions that are true right now; they clear themselves
when the condition goes away. The disarm reason is why the motor last switched
off, and it stays latched until cleared. Clearing re-arms nothing.

Exits 1 while anything is wrong, so scripts can use it as a health check.
"""

from __future__ import annotations

import argparse
import logging
import time
from collections.abc import Collection, Mapping
from dataclasses import dataclass
from typing import TYPE_CHECKING

from darm.apps._commissioning import label_width, present_nodes
from darm.odrive.node import ODriveNode, SdoTimeoutError
from darm.odrive.protocol import describe_errors

if TYPE_CHECKING:
    from darm.cli import Context

log = logging.getLogger(__name__)

_SETTLE_S = 0.5


@dataclass(frozen=True, slots=True)
class Report:
    state: str
    active: int | None  # None when the node did not answer
    disarm_reason: int | None

    @property
    def clean(self) -> bool:
        return self.active == 0 and self.disarm_reason == 0


def register(commands: argparse._SubParsersAction[argparse.ArgumentParser]) -> None:
    parser = commands.add_parser(
        "errors",
        help="show ODrive errors, and clear them",
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--clear", action="store_true", help="clear latched errors, then show what remains")
    parser.set_defaults(run=run)


def run(args: argparse.Namespace, ctx: Context) -> int:
    config = ctx.config
    with ctx.bus(config.node_ids) as bus:
        nodes = present_nodes(bus, config.node_ids)
        if args.clear:
            _clear(nodes.values())
        reports = {node_id: _report(node) for node_id, node in nodes.items()}
    missing = sorted(set(config.node_ids) - set(nodes))
    _print(reports, missing, config.node_labels())
    return 0 if reports and not missing and all(r.clean for r in reports.values()) else 1


def _clear(nodes: Collection[ODriveNode]) -> None:
    """Clear every node, then wait for a heartbeat sent after the clear, so the report shows its effect."""
    cleared_at = time.monotonic()
    for node in nodes:
        node.clear_errors()
    for node in nodes:
        node.wait_until(lambda n: n.heartbeat is not None and n.heartbeat.at > cleared_at, _SETTLE_S)


def _report(node: ODriveNode) -> Report:
    try:
        errors = node.request_errors()
    except SdoTimeoutError:
        return Report(node.state.name, None, None)
    return Report(node.state.name, errors.active, errors.disarm_reason)


def _print(reports: Mapping[int, Report], missing: list[int], labels: Mapping[int, str]) -> None:
    width = label_width(labels)
    print(f"{'node':<5} {'drives':<{width}}  {'state':<22} {'active errors':<30} disarm reason")
    for node_id, report in reports.items():
        active = "no answer" if report.active is None else describe_errors(report.active)
        disarm = "no answer" if report.disarm_reason is None else describe_errors(report.disarm_reason)
        print(f"{node_id:<5} {labels.get(node_id, ''):<{width}}  {report.state:<22} {active:<30} {disarm}")
    for node_id in missing:
        print(f"{node_id:<5} {labels.get(node_id, ''):<{width}}  silent")
