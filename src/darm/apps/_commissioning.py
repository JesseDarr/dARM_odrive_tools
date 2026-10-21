"""Steps the commissioning tools share: finding nodes, asking first, saving safely.

These tools change what an ODrive does at its next power-up, so they are all
conservative in the same ways. They only touch nodes that answered, run the
expected firmware and sit idle. They ask before acting. And they never assume a
save happened without watching the node go quiet and come back.
"""

from __future__ import annotations

import logging
import time
from collections.abc import Callable, Iterable, Mapping

from darm.config import RobotConfig
from darm.odrive.bus import CanBus
from darm.odrive.endpoints import EndpointTable
from darm.odrive.node import ODriveNode, SdoTimeoutError
from darm.odrive.protocol import AxisState, RebootAction

log = logging.getLogger(__name__)

FACTORY_HEARTBEAT_S = 0.1  # What an ODrive sends at until something configures it.
_LISTEN_S = 0.6
_REBOOT_TIMEOUT_S = 5.0
_POLL_S = 0.01


def confirm(question: str, assume_yes: bool) -> bool:
    """Ask a yes/no question. Anything but an explicit yes, including having no terminal, is no."""
    if assume_yes:
        return True
    try:
        answer = input(f"{question} [y/N] ")
    except EOFError:
        return False
    return answer.strip().lower() in ("y", "yes")


def label_width(labels: Mapping[int, str]) -> int:
    return max((len(label) for label in labels.values()), default=0)


def heartbeat_period(config: RobotConfig) -> float:
    return config.telemetry_ms["heartbeat_ms"] / 1000


def present_nodes(
    bus: CanBus,
    node_ids: Iterable[int],
    endpoints: EndpointTable | None = None,
    listen_s: float = _LISTEN_S,
) -> dict[int, ODriveNode]:
    """The nodes among node_ids that sent a heartbeat within listen_s, by id. Silent ones are logged."""
    nodes = {node_id: ODriveNode(bus, node_id, endpoints) for node_id in sorted(set(node_ids))}
    deadline = time.monotonic() + listen_s
    while time.monotonic() < deadline and not all(node.heartbeat for node in nodes.values()):
        time.sleep(_POLL_S)
    for node_id, node in nodes.items():
        if node.heartbeat is None:
            log.warning("node %d is silent", node_id)
    return {node_id: node for node_id, node in nodes.items() if node.heartbeat}


def firmware_problems(nodes: Iterable[ODriveNode], expected_base: str) -> list[str]:
    """Using another firmware's endpoint table writes values into the wrong settings, so any mismatch is fatal."""
    problems = []
    for node in nodes:
        try:
            firmware = node.request_version().firmware
        except SdoTimeoutError as error:
            problems.append(str(error))
            continue
        if firmware != expected_base:
            problems.append(f"node {node.node_id} runs firmware {firmware}, expected {expected_base}")
    return problems


def busy_problems(nodes: Iterable[ODriveNode]) -> list[str]:
    """Anything but IDLE means the motor is holding or calibrating: no time to reconfigure it."""
    return [
        f"node {node.node_id} is in {node.state.name}; park or release the arm first"
        for node in nodes
        if node.state is not AxisState.IDLE
    ]


def save_and_reboot(node: ODriveNode, heartbeat_s: float) -> bool:
    """Persist the node's configuration. The ODrive saves, then reboots. True once it visibly did both."""
    node.reboot(RebootAction.SAVE_CONFIGURATION)
    return await_reboot(lambda: node.heartbeat.at if node.heartbeat else None, heartbeat_s)


def await_reboot(
    last_heartbeat: Callable[[], float | None],
    heartbeat_s: float,
    timeout: float = _REBOOT_TIMEOUT_S,
) -> bool:
    """True once a node goes quiet for several heartbeat periods and then speaks again.

    A heartbeat alone proves nothing, because one may already be in flight when
    the reboot is requested. The silence is what shows the node really went down.
    """
    gap = 3 * max(heartbeat_s, FACTORY_HEARTBEAT_S)
    deadline = time.monotonic() + timeout
    quiet_since: float | None = None
    while (now := time.monotonic()) < deadline:
        last = last_heartbeat()
        if quiet_since is None:
            if last is None or now - last > gap:
                quiet_since = now
        elif last is not None and last > quiet_since:
            return True
        time.sleep(_POLL_S)
    return False
