"""darm console: drive one joint at a time from the keyboard.

Keys pick a joint and nudge its target. They never touch the motors: every
nudge goes through the arm, which clamps it to limits, slews it at the joint's
safe speed, and ignores it entirely while frozen.
"""

from __future__ import annotations

import argparse
import logging
import math
import time
from collections.abc import Sequence
from dataclasses import dataclass
from typing import TYPE_CHECKING

import urwid

from darm.apps import tui
from darm.arm.arm import Arm
from darm.config import JointSpec

if TYPE_CHECKING:
    from darm.cli import Context

log = logging.getLogger(__name__)

STEPS = {"deg": (1.0, 10.0), "mm": (0.5, 5.0)}  # unit -> (small, large)
_NUDGE_KEYS = {"up": (1, False), "down": (-1, False), "page up": (1, True), "page down": (-1, True)}
_HELP = f"left/right joint   up/down small step   pgup/pgdn large step   {tui.OPERATOR_HELP}"


def register(commands: argparse._SubParsersAction[argparse.ArgumentParser]) -> None:
    parser = commands.add_parser(
        "console",
        help="move joints from the keyboard",
        description="Engage the arm and nudge one joint at a time. Every move is clamped, slewed and supervised.",
    )
    parser.set_defaults(run=run)


@dataclass(frozen=True, slots=True)
class Nudge:
    joint: str
    delta: float


class JointKeys:
    """Keyboard -> which joint is selected and how far to nudge it."""

    def __init__(self, joints: Sequence[JointSpec]) -> None:
        if not joints:
            raise ValueError("no joints to control")
        self._joints = list(joints)
        self.selected_index = 0

    @property
    def selected(self) -> JointSpec:
        return self._joints[self.selected_index]

    def handle(self, key: str) -> Nudge | tui.Action | None:
        if key in ("left", "right"):
            step = 1 if key == "right" else -1
            self.selected_index = max(0, min(len(self._joints) - 1, self.selected_index + step))
            return None
        if key in _NUDGE_KEYS:
            direction, large = _NUDGE_KEYS[key]
            small_step, large_step = STEPS[self.selected.unit]
            return Nudge(self.selected.name, direction * (large_step if large else small_step))
        return tui.OPERATOR_KEYS.get(key)


class _JointColumn:
    def __init__(self, spec: JointSpec) -> None:
        self._spec = spec
        self._text = urwid.Text("")
        self.widget = urwid.AttrMap(urwid.LineBox(self._text), None)

    def refresh(self, measured: float, target: float, selected: bool) -> None:
        spec = self._spec
        self._text.set_text(
            [
                ("title", f"{spec.name}\n"),
                f"T {_signed(target)}\n",
                ("warn" if not math.isfinite(measured) else "ok", f"M {_signed(measured)}\n"),
                f"{spec.unit} {spec.min:g}..{spec.max:g}",
            ]
        )
        tui.highlight(self.widget, "selected" if selected else None)


class ConsoleView:
    def __init__(self, arm: Arm, keys: JointKeys, session: tui.OperatorSession, panel: tui.LogPanel) -> None:
        self._arm = arm
        self._keys = keys
        self._session = session
        self._panel = panel
        self._columns = [_JointColumn(arm.joint(name)) for name in arm.joint_names]
        self._table = tui.TelemetryTable(arm, show_joints=False)
        self._banner = tui.StatusBanner(arm, session)
        body = urwid.Pile(
            [
                urwid.Columns([column.widget for column in self._columns]),
                urwid.Divider(),
                tui.boxed(self._table.widget, "ODrives"),
                tui.boxed(panel.widget, "log"),
            ]
        )
        self.root = urwid.Frame(urwid.Filler(body, valign="top"), header=self._banner.widget, footer=urwid.Text(_HELP))

    def refresh(self) -> None:
        measured = self._arm.measured()
        targets = self._arm.targets()
        names = self._arm.joint_names
        for index, (name, column) in enumerate(zip(names, self._columns, strict=True)):
            selected = index == self._keys.selected_index
            column.refresh(measured.get(name, math.nan), targets.get(name, math.nan), selected)
        self._table.refresh(time.monotonic())
        self._banner.refresh()
        self._panel.refresh()


def run(args: argparse.Namespace, ctx: Context) -> int:
    with ctx.arm() as arm:
        arm.engage()
        session = tui.OperatorSession(arm)
        keys = JointKeys([arm.joint(name) for name in arm.joint_names])
        panel = tui.LogPanel()
        view = ConsoleView(arm, keys, session, panel)

        def on_key(key: str) -> None:
            command = keys.handle(key)
            if isinstance(command, Nudge):
                if not session.busy:
                    arm.nudge(command.joint, command.delta)  # The arm bounds how far ahead key repeat can run.
            elif command is not None and not session.perform(command):
                raise urwid.ExitMainLoop()

        def refresh() -> None:
            view.refresh()
            if session.parked:
                raise urwid.ExitMainLoop()

        try:
            tui.run(view.root, on_key, [(tui.REFRESH_S, refresh)], panel)
        except Exception:
            arm.freeze("console crashed")
            raise
        finally:
            session.shutdown()
        log.info(tui.summary(arm, session))
    return 0


def _signed(value: float) -> str:
    return f"{value:+.1f}" if math.isfinite(value) else "-"
