"""SDO and polling against a hand-driven fake ODrive, to replay exactly the frame orders that broke the old code."""

from __future__ import annotations

import struct
import threading
import uuid
from collections.abc import Callable, Iterator
from typing import Any

import can
import pytest

from darm.odrive import protocol as p
from darm.odrive.bus import CanBus
from darm.odrive.endpoints import EndpointTable
from darm.odrive.node import ODriveNode, SdoTimeoutError

NODE = 3


class FakeODrive:
    """The other end of the wire, driven by the test one frame at a time."""

    def __init__(self, channel: str) -> None:
        self._bus = can.Bus(interface="virtual", channel=channel)

    def expect(self, cmd: p.Cmd) -> can.Message:
        message = self._bus.recv(timeout=1.0)
        assert message is not None, f"host never sent {cmd.name}"
        assert p.split_arbitration_id(message.arbitration_id) == (NODE, cmd)
        return message

    def send(self, cmd: p.Cmd, data: bytes) -> None:
        self._bus.send(can.Message(arbitration_id=p.arbitration_id(NODE, cmd), data=data, is_extended_id=False))

    def reply_sdo(self, endpoint_id: int, value: bytes = b"") -> None:
        self.send(p.Cmd.TX_SDO, struct.pack("<BHB", 0, endpoint_id, 0) + value)

    def close(self) -> None:
        self._bus.shutdown()


@pytest.fixture
def wire() -> Iterator[tuple[ODriveNode, FakeODrive]]:
    channel = f"node-{uuid.uuid4().hex}"
    fake = FakeODrive(channel)
    bus = CanBus(channel, "virtual", [NODE])
    yield ODriveNode(bus, NODE, EndpointTable.for_firmware("0.6.12-1")), fake
    bus.close()
    fake.close()


def in_background(call: Callable[[], Any]) -> Callable[[], Any]:
    """Run call on a thread; the returned function joins it and returns its result or raises its exception."""
    outcome: dict[str, Any] = {}

    def target() -> None:
        try:
            outcome["value"] = call()
        except Exception as error:  # Re-raised on the test thread.
            outcome["error"] = error

    thread = threading.Thread(target=target)
    thread.start()

    def join() -> Any:
        thread.join(timeout=2.0)
        if "error" in outcome:
            raise outcome["error"]
        return outcome["value"]

    return join


def test_a_read_returns_the_decoded_value(wire: tuple[ODriveNode, FakeODrive]) -> None:
    node, fake = wire
    result = in_background(lambda: node.read("axis0.config.motor.pole_pairs"))
    request = fake.expect(p.Cmd.RX_SDO)
    _, endpoint_id, _ = struct.unpack_from("<BHB", request.data)
    fake.reply_sdo(endpoint_id, struct.pack("<I", 20))
    assert result() == 20


def test_late_write_acks_and_foreign_replies_are_never_taken_as_the_answer(
    wire: tuple[ODriveNode, FakeODrive],
) -> None:
    node, fake = wire
    endpoint = node.endpoints["axis0.config.init_pos"]  # type: ignore[index]
    result = in_background(lambda: node.read("axis0.config.init_pos", timeout=1.0))
    fake.expect(p.Cmd.RX_SDO)
    fake.reply_sdo(endpoint.id)  # A stale empty write ack for the same endpoint.
    fake.reply_sdo(endpoint.id + 1, struct.pack("<f", 1.0))  # Someone else's answer.
    fake.reply_sdo(endpoint.id, struct.pack("<f", 0.25))
    assert result() == 0.25


def test_a_read_with_no_matching_reply_times_out(wire: tuple[ODriveNode, FakeODrive]) -> None:
    node, fake = wire
    result = in_background(lambda: node.read("axis0.config.init_pos", timeout=0.2))
    fake.expect(p.Cmd.RX_SDO)
    fake.reply_sdo(1, struct.pack("<f", 1.0))
    with pytest.raises(SdoTimeoutError):
        result()


def test_a_write_waits_for_the_firmware_ack(wire: tuple[ODriveNode, FakeODrive]) -> None:
    node, fake = wire
    result = in_background(lambda: node.write("axis0.config.motor.pole_pairs", 21))
    request = fake.expect(p.Cmd.RX_SDO)
    opcode, endpoint_id, _, value = struct.unpack_from("<BHBI", request.data)
    assert (opcode, value) == (1, 21)
    fake.reply_sdo(endpoint_id)
    assert result() is None


def test_writes_to_read_only_endpoints_never_reach_the_wire(wire: tuple[ODriveNode, FakeODrive]) -> None:
    node, _ = wire
    with pytest.raises(PermissionError, match="read-only"):
        node.write("axis0.current_state", 8)


def test_version_is_requested_with_a_remote_frame(wire: tuple[ODriveNode, FakeODrive]) -> None:
    node, fake = wire
    result = in_background(node.request_version)
    request = fake.expect(p.Cmd.GET_VERSION)
    assert request.is_remote_frame
    fake.send(p.Cmd.GET_VERSION, bytes([2, 5, 2, 0, 0, 6, 12, 0]))
    assert result().firmware == "0.6.12"


def test_malformed_frames_are_dropped_without_killing_the_reader(wire: tuple[ODriveNode, FakeODrive]) -> None:
    node, fake = wire
    fake.send(p.Cmd.HEARTBEAT, b"\x01")
    fake.send(p.Cmd.HEARTBEAT, struct.pack("<IBBB", 0, 1, 0, 1))
    assert node.wait_until(lambda n: n.heartbeat is not None, timeout=1.0)
    assert node.state is p.AxisState.IDLE
