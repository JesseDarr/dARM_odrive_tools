"""darm gamepad: drive the arm with a game controller.

Nothing moves unless the dead-man button is held. The stick mapping lives in
darm.teleop, which is pure and tested; this module only reads the controller,
feeds the arm a nudge per tick, and freezes the arm the moment the controller
goes away. Everything runs on urwid's thread, so pygame is only ever touched
from the thread that started it.
"""

from __future__ import annotations

import argparse
import logging
import os
import time
from types import ModuleType
from typing import TYPE_CHECKING, Any

import urwid

from darm import teleop
from darm.apps import tui
from darm.arm.arm import Arm
from darm.config import GamepadSpec

if TYPE_CHECKING:
    from darm.cli import Context

log = logging.getLogger(__name__)

_MAX_TICKS_PER_GAP = 2  # A stalled tick never integrates more than this many periods of stick travel.


def register(commands: argparse._SubParsersAction[argparse.ArgumentParser]) -> None:
    parser = commands.add_parser(
        "gamepad",
        help="drive the arm with a game controller",
        description="Engage the arm and jog joints from a controller. Nothing moves unless the dead-man is held.",
    )
    parser.add_argument("--index", type=int, default=0, help="which controller, if several are connected")
    parser.set_defaults(run=run)


class Gamepad:
    """One pygame joystick: its axes and buttons, or None for good once it has gone away."""

    def __init__(self, pygame: ModuleType, joystick: Any) -> None:
        self._pygame = pygame
        self._joystick = joystick
        self._instance_id = joystick.get_instance_id()
        self.name: str = joystick.get_name()
        self.connected = True

    def read(self) -> tuple[list[float], list[bool]] | None:
        for event in self._pygame.event.get():
            removed = event.type == self._pygame.JOYDEVICEREMOVED
            if removed and getattr(event, "instance_id", None) == self._instance_id:
                self.connected = False
        if not self.connected:
            return None
        try:
            axes = [float(self._joystick.get_axis(i)) for i in range(self._joystick.get_numaxes())]
            buttons = [bool(self._joystick.get_button(i)) for i in range(self._joystick.get_numbuttons())]
        except self._pygame.error:
            self.connected = False
            return None
        return axes, buttons


class GamepadDriver:
    """Turns one controller reading per tick into arm nudges."""

    def __init__(self, arm: Arm, pad: Gamepad, spec: GamepadSpec, session: tui.OperatorSession) -> None:
        self._arm = arm
        self._pad = pad
        self._spec = spec
        self._session = session
        self._max_speeds = {name: arm.joint(name).max_speed for name in arm.joint_names}
        self._max_dt = _MAX_TICKS_PER_GAP / spec.rate_hz
        self._last = time.monotonic()
        self.axes: list[float] = []
        self.buttons: list[bool] = []
        self.velocities: dict[str, float] = {}
        self.lost = False

    def tick(self) -> None:
        now = time.monotonic()
        dt = min(now - self._last, self._max_dt)
        self._last = now
        if self.lost:
            return
        reading = self._pad.read()
        if reading is None:
            self._lose_controller()
            return
        self.axes, self.buttons = reading
        self.velocities = teleop.joint_velocities(self.axes, self.buttons, self._spec, self._max_speeds)
        if self._session.busy:
            return
        for joint, velocity in self.velocities.items():
            self._arm.nudge(joint, velocity * dt)

    def _lose_controller(self) -> None:
        self.lost = True
        self.velocities = {}
        self._arm.freeze("gamepad disconnected")
        log.error("gamepad %s disconnected; arm frozen. Reconnect and restart the tool to drive again.", self._pad.name)


class GamepadView:
    def __init__(
        self, arm: Arm, driver: GamepadDriver, session: tui.OperatorSession, panel: tui.LogPanel, spec: GamepadSpec
    ) -> None:
        self._arm = arm
        self._driver = driver
        self._spec = spec
        self._panel = panel
        self._controller = urwid.Text("")
        self._table = tui.TelemetryTable(arm, show_joints=True)
        self._banner = tui.StatusBanner(arm, session)
        body = urwid.Pile(
            [
                tui.boxed(self._controller, "controller"),
                tui.boxed(self._table.widget, "ODrives and joints"),
                tui.boxed(panel.widget, "log"),
            ]
        )
        hold = f"hold button {spec.deadman_button} to move, {spec.shift_button} too for the wrist"
        help_text = f"{hold}   {tui.OPERATOR_HELP}"
        self.root = urwid.Frame(
            urwid.Filler(body, valign="top"), header=self._banner.widget, footer=urwid.Text(help_text)
        )

    def refresh(self) -> None:
        self._controller.set_text(controller_markup(self._driver, self._spec, self._arm))
        self._table.refresh(time.monotonic())
        self._banner.refresh()
        self._panel.refresh()


def controller_markup(driver: GamepadDriver, spec: GamepadSpec, arm: Arm) -> tui.Markup:
    if driver.lost:
        return [("bad", "controller disconnected: arm frozen")]
    held = teleop.is_pressed(driver.buttons, spec.deadman_button)
    shifted = teleop.is_pressed(driver.buttons, spec.shift_button)
    moving = ", ".join(
        f"{joint} {velocity:+.0f} {arm.joint(joint).unit}/s" for joint, velocity in driver.velocities.items()
    )
    axes = "  ".join(f"{index}:{value:+.2f}" for index, value in enumerate(driver.axes))
    pressed = " ".join(str(index) for index, down in enumerate(driver.buttons) if down) or "none"
    return [
        ("ok", "dead-man HELD") if held else ("dim", "dead-man released"),
        f"   shift {'on' if shifted else 'off'}   moving: {moving or 'nothing'}\n",
        f"axes     {axes}\nbuttons  {pressed}",
    ]


def run(args: argparse.Namespace, ctx: Context) -> int:
    pygame = _start_pygame()
    if pygame is None:
        return 1
    try:
        pad = _open(pygame, args.index, ctx.config.gamepad)
        if pad is None:
            return 1
        with ctx.arm() as arm:
            arm.engage()
            _drive(arm, pad, ctx.config.gamepad)
        return 0
    finally:
        pygame.quit()


def _drive(arm: Arm, pad: Gamepad, spec: GamepadSpec) -> None:
    session = tui.OperatorSession(arm)
    driver = GamepadDriver(arm, pad, spec, session)
    panel = tui.LogPanel()
    view = GamepadView(arm, driver, session, panel, spec)

    def on_key(key: str) -> None:
        action = tui.OPERATOR_KEYS.get(key)
        if action is not None and not session.perform(action):
            raise urwid.ExitMainLoop()

    def refresh() -> None:
        view.refresh()
        if session.parked:
            raise urwid.ExitMainLoop()

    try:
        tui.run(view.root, on_key, [(1.0 / spec.rate_hz, driver.tick), (tui.REFRESH_S, refresh)], panel)
    except Exception:
        arm.freeze("gamepad tool crashed")
        raise
    finally:
        session.shutdown()
    log.info(tui.summary(arm, session))


def _start_pygame() -> ModuleType | None:
    """Import pygame only when this tool runs, headless-friendly, so the rest of darm works without it."""
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    os.environ.setdefault("SDL_AUDIODRIVER", "dummy")
    os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
    try:
        import pygame  # noqa: PLC0415  # Optional dependency: only this tool needs it.
    except ImportError:
        log.error("the gamepad tool needs pygame: pip install 'darm[gamepad]'")
        return None
    pygame.init()
    pygame.joystick.init()
    module: ModuleType = pygame
    return module


def _open(pygame: ModuleType, index: int, spec: GamepadSpec) -> Gamepad | None:
    """Find the controller before anything is armed: no controller, no arm."""
    count = pygame.joystick.get_count()
    if not 0 <= index < count:
        log.error("no gamepad %d (%d connected). Pair it, then check /dev/input/js*.", index, count)
        return None
    joystick = pygame.joystick.Joystick(index)
    joystick.init()
    axes, buttons = joystick.get_numaxes(), joystick.get_numbuttons()
    log.info("gamepad: %s (%d axes, %d buttons)", joystick.get_name(), axes, buttons)
    if spec.deadman_button >= buttons:
        log.error("dead-man button %d does not exist on this controller; nothing could ever move", spec.deadman_button)
        return None
    for binding in spec.bindings:
        if binding.axis >= axes:
            log.warning("binding for %s uses axis %d, which this controller lacks", binding.joint, binding.axis)
    return Gamepad(pygame, joystick)
