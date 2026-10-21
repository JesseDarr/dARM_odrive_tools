"""darm address: find every ODrive on the bus by serial number and give each its node id.

A firmware update erases an ODrive's node id along with the rest of its
configuration. It comes back unaddressed: silent, invisible to everything else
in darm, answering only the Address message. Run this first after flashing:

  darm address --list                  who is on the bus
  darm address                         blink each ODrive in turn and ask which node it is
  darm address --assign SERIAL=NODE    the same, without asking

Every change goes to an ODrive this run just found, by its serial number. Each
new id is saved and then confirmed by watching the ODrive reboot under it.
"""

from __future__ import annotations

import argparse
import logging
from collections.abc import Mapping
from typing import TYPE_CHECKING, Literal

from darm.apps._commissioning import await_reboot, heartbeat_period
from darm.odrive.addressing import AddressBook, assign, enumerate_odrives, free_node_id, identify, save
from darm.odrive.bus import CanBus
from darm.odrive.protocol import BROADCAST_NODE_ID, MAX_NODE_ID, AddressReply, AxisState

if TYPE_CHECKING:
    from darm.cli import Context

log = logging.getLogger(__name__)

Answer = int | None | Literal["quit"]  # node id, skip, or stop asking


def register(commands: argparse._SubParsersAction[argparse.ArgumentParser]) -> None:
    parser = commands.add_parser(
        "address",
        help="find ODrives by serial number and assign node ids (run after flashing)",
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--list", action="store_true", help="only list what is on the bus")
    mode.add_argument(
        "--assign", nargs="+", type=_assignment, metavar="SERIAL=NODE", help="assign without asking (hex serial)"
    )
    parser.set_defaults(run=run)


def run(args: argparse.Namespace, ctx: Context) -> int:
    labels = ctx.config.node_labels()
    with ctx.bus() as bus:
        book = AddressBook(bus)
        found = enumerate_odrives(bus, book)
        if not found:
            log.error("no ODrives answered. Is the CAN interface up and the power box on?")
            return 1
        _print_table(found, labels)
        if args.list:
            return 0
        addresser = Addresser(bus, book, found, heartbeat_period(ctx.config))
        if args.assign:
            serials = [serial for serial, _ in args.assign]
            if len(set(serials)) != len(serials):
                log.error("an ODrive is listed more than once")
                return 1
            ok = addresser.apply(dict(args.assign), labels)
        else:
            ok = _ask_each(addresser, found, labels)
        print()
        _print_table(enumerate_odrives(bus, book), labels)
    return 0 if ok else 1


class Addresser:
    """Changes node ids one ODrive at a time, tracking who holds which id as it goes.

    Giving an ODrive an id another one holds makes the other give it up and go
    unaddressed. Tracking that here is what keeps a later step from blinking,
    or re-addressing, the wrong board.
    """

    def __init__(self, bus: CanBus, book: AddressBook, found: Mapping[int, AddressReply], heartbeat_s: float) -> None:
        self._bus = bus
        self._book = book
        self._heartbeat_s = heartbeat_s
        self._temporary: set[int] = set()  # Serials holding an unsaved id we gave them to blink by.
        self.ids = {serial: reply.node_id for serial, reply in found.items()}

    def holder(self, node_id: int) -> int | None:
        return next((serial for serial, held in self.ids.items() if held == node_id), None)

    def apply(self, plan: Mapping[int, int], labels: Mapping[int, str]) -> bool:
        """Assign every SERIAL -> NODE in plan, or refuse all of it if any part looks wrong."""
        problems = self._plan_problems(plan)
        for problem in problems:
            log.error("%s", problem)
        if problems:
            return False
        ok = True
        for serial, node_id in plan.items():
            if node_id not in labels:
                log.warning("node %d is not in the config", node_id)
            ok = self.give(serial, node_id) and ok
        return ok

    def give(self, serial: int, node_id: int) -> bool:
        """Assign node_id to this ODrive, save it, and confirm it rebooted under the new id."""
        current = self.ids[serial]
        if current == node_id and serial not in self._temporary:
            log.info("ODrive %012X is already node %d", serial, node_id)
            return True
        previous = self.holder(node_id)
        busy = [s for s in (serial, previous) if s is not None and not self._idle(s)]
        if busy:
            log.error("ODrive %012X (node %d) is not idle; release the arm first", busy[0], self.ids[busy[0]])
            return False
        if previous is not None and previous != serial:
            log.warning("ODrive %012X gives up node %d and is now unaddressed", previous, node_id)
            self.ids[previous] = BROADCAST_NODE_ID
        assign(self._bus, serial, node_id)
        self.ids[serial] = node_id
        self._temporary.discard(serial)
        save(self._bus, node_id)
        if not await_reboot(lambda: self._last_heartbeat(node_id), self._heartbeat_s):
            log.error("ODrive %012X did not come back as node %d; its id may not be saved", serial, node_id)
            return False
        print(f"ODrive {serial:012X} is now node {node_id}, saved.")
        return True

    def blinkable_id(self, serial: int) -> int:
        """An id to blink this ODrive by. An unaddressed one gets a temporary, unsaved id first."""
        if self.ids[serial] == BROADCAST_NODE_ID:
            temporary = free_node_id(self.ids.values())
            assign(self._bus, serial, temporary)
            self.ids[serial] = temporary
            self._temporary.add(serial)
        return self.ids[serial]

    def has_temporary_id(self, serial: int) -> bool:
        return serial in self._temporary

    def identify(self, node_id: int, on: bool) -> None:
        identify(self._bus, node_id, on)

    def _plan_problems(self, plan: Mapping[int, int]) -> list[str]:
        problems = [f"ODrive {serial:012X} did not answer this run" for serial in plan if serial not in self.ids]
        targets = list(plan.values())
        problems += [f"node {n} is assigned twice" for n in sorted(set(targets)) if targets.count(n) > 1]
        problems += [f"node {n} is outside 0..{MAX_NODE_ID}" for n in targets if not 0 <= n <= MAX_NODE_ID]
        for serial, node_id in plan.items():
            holder = self.holder(node_id)
            if holder is not None and holder != serial and holder not in plan:
                problems.append(f"node {node_id} belongs to ODrive {holder:012X}; reassign it in the same command")
        return problems

    def _idle(self, serial: int) -> bool:
        """Unaddressed, or only given an id by us just now, means nothing has commanded it. Else ask its heartbeat."""
        node_id = self.ids[serial]
        if node_id == BROADCAST_NODE_ID or serial in self._temporary:
            return True
        sighting = self._book.sighting(node_id)
        return sighting is not None and sighting.state is AxisState.IDLE

    def _last_heartbeat(self, node_id: int) -> float | None:
        sighting = self._book.sighting(node_id)
        return sighting.at if sighting else None


def _ask_each(addresser: Addresser, found: Mapping[int, AddressReply], labels: Mapping[int, str]) -> bool:
    """Blink each ODrive, unaddressed ones first, and ask which config node it is."""
    print("Config nodes: " + ", ".join(f"{node_id} {label}" for node_id, label in labels.items()))
    ok = True
    for serial in sorted(found, key=lambda s: (found[s].node_id != BROADCAST_NODE_ID, found[s].node_id)):
        node_id = addresser.blinkable_id(serial)
        addresser.identify(node_id, True)
        try:
            answer = _ask(serial, node_id, labels)
        finally:
            addresser.identify(addresser.ids[serial], False)
        if answer == "quit":
            break
        if answer is not None:
            ok = addresser.give(serial, answer) and ok
        elif addresser.has_temporary_id(serial):
            print(f"ODrive {serial:012X} keeps temporary node {node_id} until it next powers up.")
    return ok


def _ask(serial: int, current: int, labels: Mapping[int, str]) -> Answer:
    while True:
        prompt = f"ODrive {serial:012X} is blinking (now node {current}). Which node is it? [Enter skips, q quits] "
        try:
            text = input(prompt)
        except EOFError:
            return "quit"
        text = text.strip().lower()
        if text in ("", "s", "skip"):
            return None
        if text in ("q", "quit"):
            return "quit"
        if text.isdigit() and int(text) in labels:
            return int(text)
        print(f"'{text}' is not a node in the config.")


def _print_table(found: Mapping[int, AddressReply], labels: Mapping[int, str]) -> None:
    print(f"{'serial':<14} {'node':<12} drives")
    for reply in sorted(found.values(), key=lambda r: (r.node_id, r.serial_number)):
        if reply.node_id == BROADCAST_NODE_ID:
            print(f"{reply.serial:<14} {'unaddressed':<12}")
        else:
            print(f"{reply.serial:<14} {reply.node_id:<12} {labels.get(reply.node_id, 'not in config')}")
    ids = [reply.node_id for reply in found.values() if reply.node_id != BROADCAST_NODE_ID]
    for node_id in sorted({n for n in ids if ids.count(n) > 1}):
        log.warning("node %d is claimed by more than one ODrive", node_id)


def _assignment(text: str) -> tuple[int, int]:
    serial, separator, node = text.partition("=")
    try:
        if not separator:
            raise ValueError(text)
        return int(serial, 16), int(node)
    except ValueError:
        raise argparse.ArgumentTypeError(f"expected SERIAL=NODE with a hex serial, got '{text}'") from None
