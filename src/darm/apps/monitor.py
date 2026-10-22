"""darm monitor: watch every ODrive and joint live, without the power to move anything.

It only listens. The arm is discovered but never engaged, and nothing on this
screen sends a command, so it is safe to leave running next to another tool
(each process gets its own copy of every frame on the bus).
"""

import argparse
import time
from typing import TYPE_CHECKING

import urwid

from darm.apps import tui
from darm.arm.arm import Arm

if TYPE_CHECKING:
    from darm.cli import Context

_QUIT_KEYS = ("q", "esc")


def register(commands: argparse._SubParsersAction[argparse.ArgumentParser]) -> None:
    parser = commands.add_parser(
        "monitor",
        help="live telemetry, read-only",
        description="Show every ODrive's telemetry and every joint's position. Never sends a command.",
    )
    parser.set_defaults(run=run)


class MonitorView:
    def __init__(self, arm: Arm, panel: tui.LogPanel) -> None:
        self._panel = panel
        self._table = tui.TelemetryTable(arm, show_joints=True)
        self._banner = tui.StatusBanner(arm)
        body = urwid.Pile([tui.boxed(self._table.widget, "ODrives and joints"), tui.boxed(panel.widget, "log")])
        footer = urwid.Text("q quit   (yellow = stale, red = errors)")
        self.root = urwid.Frame(urwid.Filler(body, valign="top"), header=self._banner.widget, footer=footer)

    def refresh(self) -> None:
        self._table.refresh(time.monotonic())
        self._banner.refresh()
        self._panel.refresh()


def run(args: argparse.Namespace, ctx: Context) -> int:
    with ctx.arm() as arm:
        panel = tui.LogPanel()
        view = MonitorView(arm, panel)

        def on_key(key: str) -> None:
            if key in _QUIT_KEYS:
                raise urwid.ExitMainLoop()

        tui.run(view.root, on_key, [(tui.REFRESH_S, view.refresh)], panel)
    return 0
