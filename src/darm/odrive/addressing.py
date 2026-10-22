"""Finding ODrives and giving them node ids, with the CANSimple Address message.

A freshly flashed ODrive has no node id (0x3F, "unaddressed"). It sends no
heartbeats and answers nothing, so the passive discovery everything else in
darm relies on cannot see it. The Address message is the one thing it still
answers: ask on the broadcast id and every ODrive on the bus replies with its
serial number and current id.

A serial number is the only safe handle on an ODrive without an id. An Address
message carrying serial 0 is a wildcard that every ODrive hearing it obeys, so
nothing here ever sends one.
"""

import threading
import time
from collections.abc import Iterable
from dataclasses import dataclass

from darm.odrive import protocol as p
from darm.odrive.bus import CanBus

DEFAULT_QUIET_S = 1.0  # Unaddressed ODrives answer after random delays averaging 250 ms apart.
DEFAULT_MAX_LISTEN_S = 5.0
_POLL_S = 0.01


@dataclass(frozen=True, slots=True)
class Sighting:
    at: float  # time.monotonic() of the latest heartbeat
    state: p.AxisState


class AddressBook:
    """Every Address reply and heartbeat seen on the bus, collected on the reader thread.

    Listens only to nodes nobody else subscribed to, so use it on a bus of its own.
    """

    def __init__(self, bus: CanBus) -> None:
        self._lock = threading.Lock()
        self._replies: dict[int, tuple[float, p.AddressReply]] = {}  # serial -> (received at, reply)
        self._sightings: dict[int, Sighting] = {}  # node id -> latest heartbeat
        bus.on_stray_frame(self._on_frame)

    @property
    def last_reply_at(self) -> float:
        with self._lock:
            return max((at for at, _ in self._replies.values()), default=0.0)

    def replies_since(self, since: float) -> dict[int, p.AddressReply]:
        with self._lock:
            return {serial: reply for serial, (at, reply) in self._replies.items() if at >= since}

    def sighting(self, node_id: int) -> Sighting | None:
        with self._lock:
            return self._sightings.get(node_id)

    def _on_frame(self, node_id: int, cmd: int, data: bytes, now: float) -> None:
        if cmd == p.Cmd.ADDRESS and len(data) > p.SERIAL_NUMBER_BYTES:
            reply = p.decode_address(data)
            with self._lock:
                self._replies[reply.serial_number] = (now, reply)
        elif cmd == p.Cmd.HEARTBEAT:
            state = p.decode_heartbeat(data).axis_state
            with self._lock:
                self._sightings[node_id] = Sighting(now, state)


def enumerate_odrives(
    bus: CanBus,
    book: AddressBook,
    quiet_s: float = DEFAULT_QUIET_S,
    max_s: float = DEFAULT_MAX_LISTEN_S,
) -> dict[int, p.AddressReply]:
    """Ask every ODrive on the bus who it is. Returns replies by serial number.

    Addressed ODrives answer at once. Unaddressed ones wait a random delay so
    their identical arbitration ids do not collide, so listening only stops
    after quiet_s without a new reply, or max_s in all.
    """
    asked_at = time.monotonic()
    bus.send(p.BROADCAST_NODE_ID, p.Cmd.ADDRESS, remote=True)
    while True:
        time.sleep(_POLL_S)
        now = time.monotonic()
        if now - max(asked_at, book.last_reply_at) >= quiet_s or now - asked_at >= max_s:
            return book.replies_since(asked_at)


def assign(bus: CanBus, serial_number: int, node_id: int) -> None:
    """Give node_id to the ODrive with this serial. Immediate, but lost at power-off unless saved.

    Any other ODrive currently holding node_id gives it up and becomes unaddressed.
    """
    if serial_number == 0:
        raise ValueError("serial 0 is a wildcard: every ODrive on the bus would take this id")
    if not 0 <= node_id <= p.MAX_NODE_ID:
        raise ValueError(f"node id {node_id} outside 0..{p.MAX_NODE_ID}")
    bus.send(p.BROADCAST_NODE_ID, p.Cmd.ADDRESS, p.encode_address(node_id, serial_number))


def identify(bus: CanBus, node_id: int, on: bool) -> None:
    """Blink an ODrive's LED, or stop. Also clears its errors: identify rides on Clear_Errors."""
    bus.send(node_id, p.Cmd.CLEAR_ERRORS, p.encode_clear_errors(on))


def save(bus: CanBus, node_id: int) -> None:
    """Persist configuration, node id included. The ODrive must be idle; it reboots afterwards."""
    bus.send(node_id, p.Cmd.REBOOT, p.encode_reboot(p.RebootAction.SAVE_CONFIGURATION))


def free_node_id(taken: Iterable[int]) -> int:
    """The highest id nobody uses, far from the low ids a robot config hands out."""
    used = set(taken)
    for node_id in range(p.MAX_NODE_ID, -1, -1):
        if node_id not in used:
            return node_id
    raise RuntimeError("every node id is in use")
