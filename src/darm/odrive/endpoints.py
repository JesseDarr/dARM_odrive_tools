"""The firmware's endpoint table: path -> (id, type), loaded from flat_endpoints.json.

Endpoint ids are assigned by the firmware build. Using a table from one version
against an ODrive running another writes values into the wrong settings, which
is why the table carries its firmware version and every tool checks it.
"""

import difflib
import json
import struct
from collections.abc import Iterator, Mapping
from dataclasses import dataclass
from importlib import resources
from pathlib import Path

_VALUE_FORMATS: Mapping[str, str] = {
    "bool": "?",
    "uint8": "B",
    "int8": "b",
    "uint16": "H",
    "int16": "h",
    "uint32": "I",
    "int32": "i",
    "uint64": "Q",
    "int64": "q",
    "float": "f",
}

_PACKAGED_TABLES = "darm.odrive.firmware"


class UnknownEndpointError(KeyError):
    pass


@dataclass(frozen=True, slots=True)
class Endpoint:
    path: str
    id: int
    type: str
    access: str = ""  # "r", "rw", or "" where the table predates access flags (0.6.10 and older)

    @property
    def is_function(self) -> bool:
        return self.type == "function"

    @property
    def is_read_only(self) -> bool:
        return self.access == "r"

    @property
    def size(self) -> int:
        return self._codec().size

    def encode(self, value: bool | int | float) -> bytes:
        codec = self._codec()
        if self.type == "bool":
            return codec.pack(bool(value))
        if self.type == "float":
            return codec.pack(float(value))
        if isinstance(value, float) and not value.is_integer():
            raise ValueError(f"{self.path} is {self.type}; refusing to truncate {value}")
        return codec.pack(int(value))

    def decode(self, data: bytes) -> bool | int | float:
        value: bool | int | float = self._codec().unpack_from(data)[0]
        return value

    def _codec(self) -> struct.Struct:
        fmt = _VALUE_FORMATS.get(self.type)
        if fmt is None:
            raise TypeError(f"{self.path} has type '{self.type}', which has no value")
        return struct.Struct("<" + fmt)


class EndpointTable:
    def __init__(self, fw_version: str, hw_version: str, endpoints: Mapping[str, Endpoint]) -> None:
        self.fw_version = fw_version
        self.hw_version = hw_version
        self._endpoints = dict(endpoints)

    @classmethod
    def from_json(cls, text: str) -> EndpointTable:
        raw = json.loads(text)
        endpoints = {
            path: Endpoint(path, spec["id"], spec["type"], spec.get("access", ""))
            for path, spec in raw["endpoints"].items()
        }
        return cls(raw["fw_version"], raw["hw_version"], endpoints)

    @classmethod
    def from_file(cls, path: Path) -> EndpointTable:
        return cls.from_json(path.read_text(encoding="utf-8"))

    @classmethod
    def for_firmware(cls, fw_version: str) -> EndpointTable:
        """Load the table shipped with this package for an exact firmware version."""
        name = f"flat_endpoints-{fw_version}.json"
        table_file = resources.files(_PACKAGED_TABLES) / name
        if not table_file.is_file():
            shipped = sorted(p.name for p in resources.files(_PACKAGED_TABLES).iterdir() if p.name.endswith(".json"))
            raise FileNotFoundError(f"no endpoint table for firmware {fw_version}; shipped: {', '.join(shipped)}")
        return cls.from_json(table_file.read_text(encoding="utf-8"))

    def __getitem__(self, path: str) -> Endpoint:
        try:
            return self._endpoints[path]
        except KeyError:
            hints = difflib.get_close_matches(path, self._endpoints, n=3)
            hint = f" Did you mean: {', '.join(hints)}?" if hints else ""
            raise UnknownEndpointError(f"firmware {self.fw_version} has no endpoint '{path}'.{hint}") from None

    def __contains__(self, path: object) -> bool:
        return path in self._endpoints

    def __iter__(self) -> Iterator[Endpoint]:
        return iter(self._endpoints.values())
