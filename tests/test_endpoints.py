import math
import re

import pytest

from darm.odrive.endpoints import Endpoint, EndpointTable, UnknownEndpointError


@pytest.fixture(scope="module")
def table() -> EndpointTable:
    return EndpointTable.for_firmware("0.6.12-1")


def test_shipped_table_matches_the_firmware_it_is_named_for(table: EndpointTable) -> None:
    assert table.fw_version == "0.6.12-1"
    assert table.hw_version == "5.2.0"


def test_ids_come_from_the_firmware_build(table: EndpointTable) -> None:
    node_id = table["axis0.config.can.node_id"]
    assert (node_id.id, node_id.type, node_id.is_read_only) == (282, "uint32", False)


def test_ids_differ_between_firmware_versions_which_is_why_versions_are_checked(table: EndpointTable) -> None:
    old = EndpointTable.for_firmware("0.6.10")
    assert old["axis0.config.can.node_id"].id != table["axis0.config.can.node_id"].id


def test_read_only_endpoints_are_flagged(table: EndpointTable) -> None:
    assert table["axis0.current_state"].is_read_only
    assert table["axis0.active_errors"].is_read_only


def test_unknown_paths_suggest_close_matches(table: EndpointTable) -> None:
    with pytest.raises(UnknownEndpointError, match=re.escape("Did you mean: axis0.config.enable_watchdog")):
        table["axis0.config.enable_watchdgo"]


def test_values_round_trip_through_their_wire_types() -> None:
    assert Endpoint("f", 1, "float").decode(Endpoint("f", 1, "float").encode(3)) == 3.0
    assert Endpoint("b", 1, "bool").decode(Endpoint("b", 1, "bool").encode(1)) is True
    assert Endpoint("u", 1, "uint32").decode(Endpoint("u", 1, "uint32").encode(7.0)) == 7
    assert math.isnan(Endpoint("f", 1, "float").decode(Endpoint("f", 1, "float").encode(math.nan)))


def test_integers_are_never_silently_truncated() -> None:
    with pytest.raises(ValueError, match="refusing to truncate"):
        Endpoint("u", 1, "uint32").encode(2.5)


def test_functions_have_no_value() -> None:
    with pytest.raises(TypeError, match="has no value"):
        Endpoint("save_configuration", 1, "function").encode(0)


def test_missing_firmware_lists_what_is_shipped() -> None:
    with pytest.raises(FileNotFoundError, match=re.escape("0.6.12-1")):
        EndpointTable.for_firmware("9.9.9")
