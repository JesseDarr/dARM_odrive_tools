"""darm configure: make every ODrive's settings match darm.toml, then save them.

Reads before it writes, writes only what differs, reads every write back, and
saves only the nodes that changed: an ODrive reboots to save, and a needless
reboot is needless risk. Refuses to touch a node that is not idle, because
changing gains or limits under a holding motor can move the arm.
"""

from __future__ import annotations

import argparse
import logging
import math
from collections.abc import Iterable, Mapping
from dataclasses import dataclass
from typing import TYPE_CHECKING

from darm.apps._commissioning import (
    busy_problems,
    confirm,
    firmware_problems,
    heartbeat_period,
    label_width,
    present_nodes,
    save_and_reboot,
)
from darm.config import ConfigError, RobotConfig, SettingValue
from darm.odrive.endpoints import EndpointTable
from darm.odrive.node import ODriveNode, SdoTimeoutError

if TYPE_CHECKING:
    from darm.cli import Context

log = logging.getLogger(__name__)

_BAUD_RATE_SETTING = "can.config.baud_rate"
_FLOAT_TOLERANCE = 1e-6  # Relative. Floats come back as float32, so 0.106 reads as 0.10599999.


@dataclass(frozen=True, slots=True)
class Difference:
    path: str
    have: SettingValue
    want: SettingValue


def register(commands: argparse._SubParsersAction[argparse.ArgumentParser]) -> None:
    parser = commands.add_parser(
        "configure",
        help="write darm.toml's ODrive settings to the ODrives and save them",
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--node", type=int, action="append", dest="nodes", metavar="N", help="only this node (repeat)")
    parser.add_argument("--dry-run", action="store_true", help="show what would change and write nothing")
    parser.add_argument("--yes", action="store_true", help="do not ask before writing")
    parser.set_defaults(run=run)


def run(args: argparse.Namespace, ctx: Context) -> int:
    config = ctx.config
    targets = _targets(args.nodes, config)
    _check_baud_rate(config, targets)
    with ctx.bus(targets) as bus:
        nodes = present_nodes(bus, targets, EndpointTable.for_firmware(config.firmware))
        missing = sorted(set(targets) - set(nodes))
        problems = firmware_problems(nodes.values(), config.firmware_base) + busy_problems(nodes.values())
        if problems:
            for problem in problems:
                log.error("%s", problem)
            return 1
        try:
            plans = {node_id: _differences(node, config.settings_for(node_id)) for node_id, node in nodes.items()}
        except SdoTimeoutError as error:
            log.error("reading settings failed: %s", error)
            return 1
        _report(plans, config.node_labels())
        changes = sum(len(differences) for differences in plans.values())
        if args.dry_run or changes == 0:
            return 1 if missing else 0
        if not confirm(f"Write {changes} setting(s) and save (each changed node reboots)?", args.yes):
            print("Nothing written.")
            return 1
        failures = 0
        for node_id, differences in plans.items():
            if differences and not _apply(nodes[node_id], differences, config):
                failures += 1
    return 1 if failures or missing else 0


def _targets(requested: list[int] | None, config: RobotConfig) -> tuple[int, ...]:
    if not requested:
        return config.node_ids
    unknown = sorted(set(requested) - set(config.node_ids))
    if unknown:
        raise ConfigError(f"node(s) {unknown} are not in the config")
    return tuple(sorted(set(requested)))


def _check_baud_rate(config: RobotConfig, targets: Iterable[int]) -> None:
    """A baud rate change takes effect the moment it is written, taking the node off this bus for good."""
    for node_id in targets:
        rate = config.settings_for(node_id).get(_BAUD_RATE_SETTING)
        if rate is not None and rate != config.bus.bitrate:
            raise ConfigError(
                f"{_BAUD_RATE_SETTING} = {rate} but the bus runs at {config.bus.bitrate}; node {node_id} would vanish"
            )


def _differences(node: ODriveNode, settings: Mapping[str, SettingValue]) -> list[Difference]:
    differences = []
    for path, want in settings.items():
        have = node.read(path)
        if not same(have, want):
            differences.append(Difference(path, have, want))
    return differences


def _apply(node: ODriveNode, differences: list[Difference], config: RobotConfig) -> bool:
    """Write, verify, save, then verify again after the reboot. False at the first thing that does not hold."""
    try:
        for difference in differences:
            node.write(difference.path, difference.want)
            if not same(node.read(difference.path), difference.want):
                log.error("node %d: %s did not take %s", node.node_id, difference.path, format_value(difference.want))
                return False
        if not save_and_reboot(node, heartbeat_period(config)):
            log.error("node %d did not reboot after saving; its settings may not survive a power cycle", node.node_id)
            return False
        lost = [d.path for d in differences if not same(node.read(d.path), d.want)]
    except SdoTimeoutError as error:
        log.error("node %d: %s", node.node_id, error)
        return False
    if lost:
        log.error("node %d lost %s across its reboot", node.node_id, ", ".join(lost))
        return False
    log.info("node %d: %d setting(s) written and saved", node.node_id, len(differences))
    return True


def _report(plans: Mapping[int, list[Difference]], labels: Mapping[int, str]) -> None:
    width = max((len(d.path) for diffs in plans.values() for d in diffs), default=0)
    for node_id, differences in plans.items():
        summary = f"{len(differences)} setting(s) differ" if differences else "up to date"
        print(f"node {node_id:<2} {labels.get(node_id, ''):<{label_width(labels)}}  {summary}")
        for d in differences:
            print(f"    {d.path:<{width}}  {format_value(d.have)} -> {format_value(d.want)}")


def same(have: SettingValue, want: SettingValue) -> bool:
    """Equal as the ODrive stores it: floats to float32 precision, NaN equal to NaN."""
    if isinstance(have, bool) or isinstance(want, bool):
        return bool(have) == bool(want)
    if isinstance(have, float) or isinstance(want, float):
        have_f, want_f = float(have), float(want)
        if math.isnan(have_f) or math.isnan(want_f):
            return math.isnan(have_f) and math.isnan(want_f)
        return math.isclose(have_f, want_f, rel_tol=_FLOAT_TOLERANCE)
    return have == want


def format_value(value: SettingValue) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, float):
        return f"{value:g}"
    return str(value)
