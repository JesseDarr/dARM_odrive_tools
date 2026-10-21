"""darm backup: copy every readable ODrive setting to a JSON file. Reads only, never writes.

Flashing new firmware erases an ODrive's configuration and calibration. Run
this first, naming the firmware the ODrives run now with --firmware, and every
value they hold is on disk before any of it is lost.

Values too wide for one classic CAN frame (the 64-bit serial number, for one)
cannot be read over CANSimple and are left out. A read that gets no answer is
recorded as null. Nodes are read in parallel: each has its own request channel.
"""

from __future__ import annotations

import argparse
import json
import logging
import time
from collections.abc import Sequence
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import TYPE_CHECKING

from darm.apps._commissioning import firmware_problems, present_nodes
from darm.config import SettingValue
from darm.odrive.endpoints import Endpoint, EndpointTable
from darm.odrive.node import ODriveNode, SdoTimeoutError
from darm.odrive.protocol import base_version

if TYPE_CHECKING:
    from darm.cli import Context

log = logging.getLogger(__name__)

_MAX_VALUE_BYTES = 4  # A classic 8-byte frame minus the 4-byte SDO header.
_GIVE_UP_AFTER = 10  # Consecutive unanswered reads before a node is presumed gone.


def register(commands: argparse._SubParsersAction[argparse.ArgumentParser]) -> None:
    parser = commands.add_parser(
        "backup",
        help="save every readable ODrive setting to a JSON file (do this before flashing)",
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--firmware", help="firmware the ODrives run now (default: the config's)")
    parser.add_argument("--out", type=Path, help="output file (default: darm-backup-<firmware>-<time>.json)")
    parser.set_defaults(run=run)


def run(args: argparse.Namespace, ctx: Context) -> int:
    config = ctx.config
    firmware = args.firmware or config.firmware
    try:
        table = EndpointTable.for_firmware(firmware)
    except FileNotFoundError as error:
        log.error("%s", error)
        return 1
    endpoints = [endpoint for endpoint in table if _readable(endpoint)]
    with ctx.bus(config.node_ids) as bus:
        nodes = present_nodes(bus, config.node_ids, table)
        if not nodes:
            log.error("no ODrives answered")
            return 1
        problems = firmware_problems(nodes.values(), base_version(firmware))
        if problems:
            for problem in problems:
                log.error("%s; pass the firmware they run with --firmware", problem)
            return 1
        log.info("reading %d settings from node(s) %s", len(endpoints), ", ".join(map(str, nodes)))
        with ThreadPoolExecutor(max_workers=len(nodes)) as pool:
            values = list(pool.map(lambda node: _read_all(node, endpoints), nodes.values()))
    out = args.out or Path(f"darm-backup-{firmware}-{time.strftime('%Y%m%d-%H%M%S')}.json")
    document = {
        "firmware": firmware,
        "hardware": table.hw_version,
        "created": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "nodes": {str(node_id): node_values for node_id, node_values in zip(nodes, values, strict=True)},
    }
    out.write_text(json.dumps(document, indent=2), encoding="utf-8")
    unanswered = sum(value is None for node_values in values for value in node_values.values())
    if unanswered:
        log.warning("%d read(s) got no answer and are recorded as null", unanswered)
    print(f"Backed up {len(nodes)} node(s), {len(endpoints)} settings each, to {out}")
    return 0


def _readable(endpoint: Endpoint) -> bool:
    if endpoint.is_function:
        return False
    try:
        return endpoint.size <= _MAX_VALUE_BYTES
    except TypeError:  # endpoint_ref and other types that carry no value
        return False


def _read_all(node: ODriveNode, endpoints: Sequence[Endpoint]) -> dict[str, SettingValue | None]:
    values: dict[str, SettingValue | None] = {}
    unanswered_in_a_row = 0
    for endpoint in endpoints:
        if unanswered_in_a_row >= _GIVE_UP_AFTER:
            values[endpoint.path] = None
            continue
        try:
            values[endpoint.path] = node.read(endpoint.path)
            unanswered_in_a_row = 0
        except SdoTimeoutError:
            values[endpoint.path] = None
            unanswered_in_a_row += 1
    if unanswered_in_a_row >= _GIVE_UP_AFTER:
        log.error("node %d stopped answering; the rest of its settings are null", node.node_id)
    log.info("node %d: done", node.node_id)
    return values
