"""One CAN socket, one reader thread, frames routed to the node they came from.

Nothing but the reader thread ever calls recv(). Two threads reading one socket
steal each other's frames, and every reply that lands on the wrong thread is
silently lost. Routing every frame from a single place makes that impossible.
"""

from __future__ import annotations

import logging
import threading
import time
from collections.abc import Callable, Iterable
from types import TracebackType

import can

from darm.odrive.protocol import CMD_BITS, NODE_FILTER_MASK, arbitration_id, split_arbitration_id

log = logging.getLogger(__name__)

FrameHandler = Callable[[int, bytes, float], None]
"""(cmd, data, monotonic receive time) -> None. Runs on the reader thread, so it must be quick."""

StrayFrameHandler = Callable[[int, int, bytes, float], None]
"""(node_id, cmd, data, monotonic receive time) -> None, for frames from nodes nobody subscribed to."""

_RECV_TIMEOUT_S = 0.1  # How often the reader wakes to notice close(); recv blocks in the kernel until then.
_SEND_TIMEOUT_S = 0.01
_ERROR_BACKOFF_S = 0.05


class BusError(RuntimeError):
    pass


class CanBus:
    def __init__(self, channel: str, interface: str, node_ids: Iterable[int] | None = None) -> None:
        """Open the bus. With node_ids, the kernel drops every frame from any other node."""
        filters = None if node_ids is None else [_node_filter(n) for n in sorted(set(node_ids))]
        try:
            self._bus = can.Bus(channel=channel, interface=interface, can_filters=filters)
        except (can.CanError, OSError) as error:
            raise BusError(f"cannot open CAN {interface}:{channel}: {error}") from error
        self._handlers: dict[int, FrameHandler] = {}
        self._stray_handler: StrayFrameHandler | None = None
        self._send_lock = threading.Lock()
        self._running = True
        self._reader = threading.Thread(target=self._read_forever, name="can-reader", daemon=True)
        self._reader.start()

    def subscribe(self, node_id: int, handler: FrameHandler) -> None:
        self._handlers[node_id] = handler

    def on_stray_frame(self, handler: StrayFrameHandler | None) -> None:
        """See frames from nodes nobody subscribed to. Used for discovery."""
        self._stray_handler = handler

    def send(self, node_id: int, cmd: int, data: bytes = b"", remote: bool = False) -> None:
        message = can.Message(
            arbitration_id=arbitration_id(node_id, cmd),
            data=data,
            is_extended_id=False,
            is_remote_frame=remote,
            dlc=8 if remote else len(data),
        )
        try:
            with self._send_lock:
                self._bus.send(message, timeout=_SEND_TIMEOUT_S)
        except can.CanError as error:
            raise BusError(f"send to node {node_id} cmd 0x{cmd:02X} failed: {error}") from error

    def close(self) -> None:
        self._running = False
        self._reader.join(timeout=2 * _RECV_TIMEOUT_S + 1)
        self._bus.shutdown()

    def __enter__(self) -> CanBus:
        return self

    def __exit__(self, kind: type[BaseException] | None, error: BaseException | None, tb: TracebackType | None) -> None:
        self.close()

    def _read_forever(self) -> None:
        while self._running:
            try:
                message = self._bus.recv(timeout=_RECV_TIMEOUT_S)
            except can.CanError as error:
                log.error("CAN receive failed: %s", error)
                time.sleep(_ERROR_BACKOFF_S)
                continue
            if message is None or message.is_error_frame or message.is_remote_frame or message.is_extended_id:
                continue
            self._dispatch(message.arbitration_id, bytes(message.data), time.monotonic())

    def _dispatch(self, arb_id: int, data: bytes, now: float) -> None:
        node_id, cmd = split_arbitration_id(arb_id)
        handler = self._handlers.get(node_id)
        stray_handler = self._stray_handler
        try:
            if handler is not None:
                handler(cmd, data, now)
            elif stray_handler is not None:
                stray_handler(node_id, cmd, data, now)
        except Exception:  # A bad frame must never kill the only reader.
            log.exception("handler for node %d cmd 0x%02X failed", node_id, cmd)


def _node_filter(node_id: int) -> dict[str, int | bool]:
    return {"can_id": node_id << CMD_BITS, "can_mask": NODE_FILTER_MASK, "extended": False}

