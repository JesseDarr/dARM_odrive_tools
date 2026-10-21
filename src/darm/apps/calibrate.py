"""darm calibrate: measure each motor and find its encoder offset, then save the result.

Calibration turns the motor: a few beeps, then a slow turn each way. The offset
it finds is only right if nothing pushes back, so calibrate with the arm seated
in the dock. An offset found under load surfaces later as spinout errors.

Nodes go one at a time. Success means the calibration was seen running and then
finished with SUCCESS; a stale result from an earlier run never counts. A
failed calibration is reported and never saved.
"""

from __future__ import annotations

import argparse
import logging
import time
from collections.abc import Iterable
from typing import TYPE_CHECKING

from darm.apps._commissioning import (
    busy_problems,
    confirm,
    firmware_problems,
    heartbeat_period,
    present_nodes,
    save_and_reboot,
)
from darm.config import ConfigError, RobotConfig
from darm.odrive.node import ODriveNode
from darm.odrive.protocol import AxisState, ProcedureResult, describe_errors

if TYPE_CHECKING:
    from darm.cli import Context

log = logging.getLogger(__name__)

_START_TIMEOUT_S = 2.0
_FINISH_TIMEOUT_S = 30.0
_WARNING = (
    "Calibration turns each motor a little each way. Seat the arm in the dock and keep clear.\n"
    "An offset found while a joint carries load causes spinout errors later."
)


def register(commands: argparse._SubParsersAction[argparse.ArgumentParser]) -> None:
    parser = commands.add_parser(
        "calibrate",
        help="calibrate motors and encoders, then save",
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("nodes", type=int, nargs="*", metavar="NODE", help="node ids to calibrate")
    parser.add_argument("--joint", help="calibrate every node that drives this joint")
    parser.add_argument("--yes", action="store_true", help="do not ask before turning motors")
    parser.set_defaults(run=run)


def run(args: argparse.Namespace, ctx: Context) -> int:
    config = ctx.config
    targets = _targets(args.nodes, args.joint, config)
    labels = config.node_labels()
    print(_WARNING)
    if not confirm(f"Calibrate {', '.join(f'node {n} ({labels[n]})' for n in targets)}?", args.yes):
        print("Nothing calibrated.")
        return 1
    with ctx.bus(targets) as bus:
        nodes = present_nodes(bus, targets)
        problems = (
            [f"node {n} is silent" for n in targets if n not in nodes]
            + firmware_problems(nodes.values(), config.firmware_base)
            + busy_problems(nodes.values())
            + _error_problems(nodes.values())
        )
        if problems:
            for problem in problems:
                log.error("%s", problem)
            return 1
        failures = 0
        for node_id in targets:
            if not _calibrate_and_save(nodes[node_id], heartbeat_period(config)):
                failures += 1
    return 1 if failures else 0


def _targets(nodes: list[int], joint: str | None, config: RobotConfig) -> tuple[int, ...]:
    if bool(nodes) == bool(joint):
        raise ConfigError("name the nodes to calibrate, or --joint NAME, but not both")
    if joint:
        drive = next((d for d in config.drives if joint in {j.name for j in d.joints}), None)
        if drive is None:
            raise ConfigError(f"no joint '{joint}'; joints: {', '.join(config.joints)}")
        return drive.nodes
    unknown = sorted(set(nodes) - set(config.node_ids))
    if unknown:
        raise ConfigError(f"node(s) {unknown} are not in the config")
    return tuple(dict.fromkeys(nodes))


def _error_problems(nodes: Iterable[ODriveNode]) -> list[str]:
    """A node with errors disarms the moment calibration starts, so ask for a clean slate first."""
    return [
        f"node {node.node_id} has errors ({describe_errors(node.axis_error)}); run `darm errors --clear` first"
        for node in nodes
        if node.axis_error
    ]


def _calibrate_and_save(node: ODriveNode, heartbeat_s: float) -> bool:
    log.info("node %d: calibrating", node.node_id)
    try:
        failure = _calibrate(node)
    except KeyboardInterrupt:
        node.request_state(AxisState.IDLE)  # Stop the motor before anything else.
        raise
    if failure:
        log.error("node %d: calibration failed: %s", node.node_id, failure)
        return False
    if not save_and_reboot(node, heartbeat_s):
        log.error("node %d: calibrated, but it did not reboot after saving; the result may not be kept", node.node_id)
        return False
    log.info("node %d: calibrated and saved", node.node_id)
    return True


def _calibrate(node: ODriveNode) -> str | None:
    """None on success, otherwise what went wrong."""
    asked_at = time.monotonic()
    node.request_state(AxisState.FULL_CALIBRATION_SEQUENCE)
    if not node.wait_until(lambda n: _started(n, asked_at), _START_TIMEOUT_S):
        return f"it never started ({_outcome(node)})"
    if not node.wait_until(_finished, _FINISH_TIMEOUT_S):
        node.request_state(AxisState.IDLE)
        return f"still running after {_FINISH_TIMEOUT_S:g} s, so it was stopped"
    assert node.heartbeat is not None
    return None if node.heartbeat.value.procedure_result is ProcedureResult.SUCCESS else _outcome(node)


def _started(node: ODriveNode, asked_at: float) -> bool:
    beat = node.heartbeat
    if beat is None or beat.at <= asked_at:
        return False
    return beat.value.axis_state is not AxisState.IDLE or beat.value.procedure_result is ProcedureResult.BUSY


def _finished(node: ODriveNode) -> bool:
    beat = node.heartbeat
    return (
        beat is not None
        and beat.value.axis_state is AxisState.IDLE
        and beat.value.procedure_result is not ProcedureResult.BUSY
    )


def _outcome(node: ODriveNode) -> str:
    beat = node.heartbeat
    if beat is None:
        return "no heartbeat"
    return f"{beat.value.procedure_result.name}, errors: {describe_errors(beat.value.axis_error)}"
