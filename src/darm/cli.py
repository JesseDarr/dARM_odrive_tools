"""darm: one command, every tool. `darm --help` lists them.

Each tool is a module in darm.apps exposing:

    def register(commands) -> None   # adds its subparser and sets run=<function>
    def run(args, ctx) -> int         # the tool itself; returns an exit code

Tools never open the bus themselves. They ask the Context, which is how
`darm --sim <tool>` runs any tool against simulated ODrives instead of hardware.
"""

from __future__ import annotations

import argparse
import importlib
import logging
import os
import pkgutil
import sys
from collections.abc import Iterable, Iterator
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path
from types import ModuleType

from darm import apps
from darm.arm.arm import Arm, MotionError, StartupError
from darm.arm.journal import Journal
from darm.config import ConfigError, RobotConfig, load
from darm.odrive.bus import BusError, CanBus
from darm.odrive.node import SdoTimeoutError
from darm.sim import SimNetwork

log = logging.getLogger("darm")

_SIM_CHANNEL = "darm-sim"
_EXPECTED_FAILURES = (ConfigError, StartupError, MotionError, BusError, SdoTimeoutError)


@dataclass(frozen=True)
class Context:
    config: RobotConfig
    simulate: bool
    state_dir: Path

    @contextmanager
    def bus(self, node_ids: Iterable[int] | None = None) -> Iterator[CanBus]:
        """The CAN bus, or a simulated one. With node_ids, frames from every other node are filtered out."""
        if not self.simulate:
            with CanBus(self.config.bus.channel, self.config.bus.interface, node_ids) as bus:
                yield bus
            return
        with SimNetwork(self.config, _SIM_CHANNEL), CanBus(_SIM_CHANNEL, "virtual", node_ids) as bus:
            yield bus

    @contextmanager
    def arm(self) -> Iterator[Arm]:
        """A discovered arm, not yet engaged. Leaving the block stops supervision; motors keep holding."""
        with self.bus(self.config.node_ids) as bus, Arm(self.config, bus, self.journal()) as arm:
            arm.discover()
            yield arm

    def journal(self) -> Journal:
        return Journal(self.state_dir / ("journal-sim.json" if self.simulate else "journal.json"))


def main(argv: list[str] | None = None) -> int:
    parser = _parser()
    args = parser.parse_args(argv)
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)-7s %(message)s",
        datefmt="%H:%M:%S",
    )
    try:
        ctx = Context(load(args.config), args.sim, args.state_dir)
        return int(args.run(args, ctx))
    except _EXPECTED_FAILURES as error:
        log.error("%s", error)
        return 1
    except KeyboardInterrupt:
        log.warning("interrupted")
        return 130


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="darm", description="Supervised control and commissioning of the dARM.")
    parser.add_argument(
        "--config",
        type=Path,
        default=Path(os.environ.get("DARM_CONFIG", "darm.toml")),
        help="robot description (default: $DARM_CONFIG or ./darm.toml)",
    )
    parser.add_argument("--sim", action="store_true", help="talk to simulated ODrives instead of the CAN bus")
    parser.add_argument(
        "--state-dir",
        type=Path,
        default=Path(os.environ.get("XDG_STATE_HOME", Path.home() / ".local" / "state")) / "darm",
        help="where darm remembers whether the arm was left docked",
    )
    parser.add_argument("-v", "--verbose", action="store_true", help="debug logging")
    commands = parser.add_subparsers(title="tools", metavar="<tool>", required=True)
    for tool in _tools():
        tool.register(commands)
    return parser


def _tools() -> list[ModuleType]:
    """Every module in darm.apps that defines register(); adding a tool never means editing this file."""
    found = []
    for info in sorted(pkgutil.iter_modules(apps.__path__), key=lambda m: m.name):
        module = importlib.import_module(f"{apps.__name__}.{info.name}")
        if hasattr(module, "register"):
            found.append(module)
    return found


if __name__ == "__main__":
    sys.exit(main())
