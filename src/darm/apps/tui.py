"""Shared terminal UI pieces for the interactive tools.

Everything on screen is redrawn from caches the bus reader already keeps, so a
screen full of telemetry costs no CAN traffic at all. Redraws happen only on
urwid's own thread, from alarms. Other threads never touch a widget: the
arm's control loop logs into a queue, and the next redraw picks it up.
"""

from __future__ import annotations

import contextlib
import enum
import itertools
import logging
import math
import threading
from collections import deque
from collections.abc import Callable, Hashable, Iterator, Mapping, Sequence
from contextlib import contextmanager
from typing import Any, TypeVar

import urwid

from darm.arm.arm import Arm, ArmState, MotionError, StartupError
from darm.config import DriveSpec, RobotConfig
from darm.odrive.bus import BusError
from darm.odrive.node import ODriveNode, SdoTimeoutError, Stamped
from darm.odrive.protocol import AxisState, Heartbeat, describe_errors

log = logging.getLogger(__name__)

T = TypeVar("T")
Segment = str | tuple[str, str]
Markup = list[Segment]

REFRESH_S = 0.1

PALETTE = [
    ("title", "default,bold", ""),
    ("ok", "light green", ""),
    ("warn", "yellow", ""),
    ("bad", "light red", ""),
    ("dim", "dark gray", ""),
    ("fault", "white,bold", "dark red"),
    ("armed", "black", "dark green"),
    ("idle", "black", "light gray"),
    ("selected", "black", "light gray"),
]

_STATE_LABELS = {
    AxisState.IDLE: "idle",
    AxisState.CLOSED_LOOP_CONTROL: "closed loop",
    AxisState.FULL_CALIBRATION_SEQUENCE: "calibrating",
}
_MIN_STALE_S = 0.5
_STALE_PERIODS = 3
_WORKER_JOIN_S = 3.0  # Longer than the arm takes to give up on re-arming a node.
_OPERATION_FAILURES = (MotionError, StartupError, BusError, SdoTimeoutError)


# ================================================================ operator actions


class Action(enum.Enum):
    FREEZE = "freeze"
    CLEAR = "clear fault"
    PARK = "park"
    QUIT = "quit"


OPERATOR_KEYS: Mapping[str, Action] = {
    " ": Action.FREEZE,
    "c": Action.CLEAR,
    "p": Action.PARK,
    "q": Action.QUIT,
    "esc": Action.QUIT,
}

OPERATOR_HELP = "space freeze   c clear fault   p park + exit   q quit (arm keeps holding)"


class OperatorSession:
    """What an operator can do to an engaged arm besides moving joints.

    Parking and clearing a fault each block for seconds, so they run on a worker
    thread while the screen keeps updating, one at a time. Quitting never lets
    go of the arm: anything still in progress is frozen where it is first.
    """

    def __init__(self, arm: Arm) -> None:
        self._arm = arm
        self._worker: threading.Thread | None = None
        self.activity = ""
        self.parked = False

    @property
    def busy(self) -> bool:
        return self._worker is not None and self._worker.is_alive()

    def perform(self, action: Action) -> bool:
        """Carry out an action. Returns False when the session should end."""
        if action is Action.QUIT:
            return False
        if action is Action.FREEZE:
            self._arm.freeze("operator")
        elif action is Action.CLEAR:
            self._in_background("clearing fault", self._arm.clear_fault)
        elif action is Action.PARK:
            self._in_background("parking", self._park)
        return True

    def shutdown(self) -> None:
        if self.busy:
            self._arm.freeze(f"operator quit while {self.activity}")
            assert self._worker is not None
            self._worker.join(timeout=_WORKER_JOIN_S)

    def _park(self) -> None:
        self._arm.park(progress=log.info)
        self.parked = True

    def _in_background(self, activity: str, job: Callable[[], None]) -> None:
        if self.busy:
            log.warning("still %s", self.activity)
            return
        self.activity = activity
        self._worker = threading.Thread(target=self._guarded, args=(job,), name=activity, daemon=True)
        self._worker.start()

    def _guarded(self, job: Callable[[], None]) -> None:
        try:
            job()
        except _OPERATION_FAILURES as error:
            log.error("%s failed: %s", self.activity, error)
        except Exception:
            log.exception("%s crashed", self.activity)
            self._arm.freeze(f"{self.activity} crashed")
        finally:
            self.activity = ""


def summary(arm: Arm, session: OperatorSession | None = None) -> str:
    """One line on how the arm was left, for the terminal after the screen closes."""
    if session is not None and session.parked:
        return "arm parked in the dock and released"
    if arm.state is ArmState.FROZEN:
        return f"arm frozen and holding: {arm.fault}"
    if arm.state is ArmState.ARMED:
        return "arm holding its last position (motors stay powered)"
    return "arm idle"


# ================================================================ widgets


class StatusBanner:
    """One line, always on top: what state the arm is in and, when frozen, exactly why."""

    def __init__(self, arm: Arm, session: OperatorSession | None = None) -> None:
        self._arm = arm
        self._session = session
        self._text = urwid.Text("")
        self.widget = urwid.AttrMap(self._text, "idle")

    def refresh(self) -> None:
        attr, text = status_line(self._arm, self._session)
        self._text.set_text(text)
        highlight(self.widget, attr)


def status_line(arm: Arm, session: OperatorSession | None) -> tuple[str, str]:
    activity = f"  [{session.activity}...]" if session is not None and session.activity else ""
    if arm.state is ArmState.FROZEN:
        return "fault", f" FROZEN: {arm.fault}  (c clears, q quits holding){activity}"
    if arm.state is ArmState.ARMED:
        return "armed", f" ARMED  {len(arm.joint_names)} joints under supervision{activity}"
    return "idle", f" IDLE  read-only: nothing on screen can move the arm{activity}"


class TelemetryTable:
    """Every node's latest telemetry, optionally followed by each joint's measured vs target position."""

    def __init__(self, arm: Arm, show_joints: bool = True) -> None:
        self._arm = arm
        self._show_joints = show_joints
        self._labels = node_labels(arm.config.drives)
        self._max_age = max_ages(arm.config)
        self.widget = urwid.Text("", wrap="clip")  # A narrow terminal loses the right edge, not the columns.

    def refresh(self, now: float) -> None:
        markup = node_table(self._arm.nodes, self._labels, self._max_age, now)
        if self._show_joints:
            markup += ["\n", *joint_table(self._arm)]
        self.widget.set_text(markup)


class LogPanel(logging.Handler):
    """The last few log records, shown on screen instead of being printed over it.

    emit() runs on whichever thread logged, so it only appends to a deque;
    refresh() turns the deque into text on urwid's thread.
    """

    def __init__(self, lines: int = 6) -> None:
        super().__init__(level=logging.INFO)
        self.setFormatter(logging.Formatter("%(asctime)s %(message)s", "%H:%M:%S"))
        self._records: deque[tuple[int, str]] = deque(maxlen=lines)
        self.widget = urwid.Text("")

    def emit(self, record: logging.LogRecord) -> None:
        try:
            self._records.append((record.levelno, self.format(record)))
        except Exception:
            self.handleError(record)

    def refresh(self) -> None:
        lines = list(self._records)
        markup: Markup = [(_level_attr(level), f"{text}\n") for level, text in lines]
        self.widget.set_text(markup or "")


def boxed(widget: urwid.Widget, title: str) -> urwid.Widget:
    return urwid.LineBox(widget, title=title)


def highlight(widget: urwid.AttrMap[Any], attr: str | None) -> None:
    """Show a whole widget in attr, or plainly when attr is None."""
    attr_map: dict[Hashable, Hashable] = {None: attr}
    widget.set_attr_map(attr_map)


# ================================================================ running a screen


@contextmanager
def captured_logging(panel: LogPanel) -> Iterator[None]:
    """Route all logging to the panel while the screen is up; stderr would scribble over it."""
    root = logging.getLogger()
    saved = root.handlers[:]
    root.handlers = [panel]
    try:
        yield
    finally:
        root.handlers = saved


def every(loop: urwid.MainLoop, period_s: float, callback: Callable[[], None]) -> None:
    """Call callback on urwid's thread every period_s. Raising urwid.ExitMainLoop from it ends the screen."""

    def tick(loop: urwid.MainLoop, _: object) -> None:
        callback()
        loop.set_alarm_in(period_s, tick)

    loop.set_alarm_in(0, tick)


def run(
    root: urwid.Widget,
    on_key: Callable[[str], None],
    timers: Sequence[tuple[float, Callable[[], None]]],
    panel: LogPanel,
) -> None:
    """Show root until a callback raises urwid.ExitMainLoop or the user presses ctrl-c."""
    def keys_only(key: Any) -> None:
        if isinstance(key, str):
            on_key(key)

    loop = urwid.MainLoop(root, palette=PALETTE, unhandled_input=keys_only, handle_mouse=False)
    for period_s, callback in timers:
        every(loop, period_s, callback)
    with captured_logging(panel), contextlib.suppress(KeyboardInterrupt):
        loop.run()


# ================================================================ table contents (pure)

_NODE_COLUMNS = (
    ("node", 5),
    ("joint", 11),
    ("state", 12),
    ("pos t", 9),
    ("vel t/s", 8),
    ("Iq A", 7),
    ("torque Nm", 10),
    ("bus V", 7),
    ("FET C", 6),
    ("power W", 8),
    ("age ms", 7),
)


def node_labels(drives: Sequence[DriveSpec]) -> dict[int, str]:
    """'base', 'shoulder A' / 'shoulder B', 'wrist A' / 'wrist B': which joint each node drives."""
    labels = {}
    for drive in drives:
        base = _shared_words([joint.name for joint in drive.joints])
        for index, node_id in enumerate(drive.nodes):
            labels[node_id] = f"{base} {chr(ord('A') + index)}" if len(drive.nodes) > 1 else base
    return labels


def _shared_words(names: Sequence[str]) -> str:
    """('wrist_bend', 'wrist_twist') -> 'wrist'. A lone name, or names sharing no leading word, keep the first."""
    columns = zip(*(name.split("_") for name in names), strict=False)
    shared = [words[0] for words in itertools.takewhile(lambda words: len(set(words)) == 1, columns)]
    return "_".join(shared) if len(names) > 1 and shared else names[0]


def max_ages(config: RobotConfig) -> dict[str, float]:
    """How old each kind of telemetry may get before it is shown as stale."""

    def stale_after(key: str) -> float:
        return max(_MIN_STALE_S, _STALE_PERIODS * config.telemetry_ms[key] / 1000)

    return {
        "heartbeat": config.safety.heartbeat_timeout_s,
        "encoder": config.safety.encoder_timeout_s,
        "iq": stale_after("iq_ms"),
        "torques": stale_after("torques_ms"),
        "bus_power": stale_after("bus_voltage_ms"),
        "temperature": stale_after("temperature_ms"),
        "powers": stale_after("powers_ms"),
    }


def node_table(
    nodes: Mapping[int, ODriveNode], labels: Mapping[int, str], max_age: Mapping[str, float], now: float
) -> Markup:
    header = "".join(_align(name, width, left=name in ("joint", "state")) for name, width in _NODE_COLUMNS) + "errors\n"
    markup: Markup = [("title", header)]
    for node_id in sorted(nodes):
        markup += [*_node_row(nodes[node_id], labels.get(node_id, "?"), max_age, now), "\n"]
    return markup


def joint_table(arm: Arm) -> Markup:
    measured = arm.measured()
    targets = arm.targets()
    markup: Markup = [("title", f"{'joint':<13}{'measured':>10}{'target':>10}  unit  limits\n")]
    for name in arm.joint_names:
        spec = arm.joint(name)
        value = measured.get(name, math.nan)
        position = _number(value, ".1f", 10)
        markup += [
            f"{name:<13}",
            position if math.isfinite(value) else ("warn", position),
            f"{_number(targets.get(name, math.nan), '.1f', 10)}  {spec.unit:<4}  {spec.min:g} .. {spec.max:g}\n",
        ]
    return markup


def _node_row(node: ODriveNode, label: str, max_age: Mapping[str, float], now: float) -> Markup:
    heartbeat = node.heartbeat
    widths = dict(_NODE_COLUMNS)
    row: Markup = [_align(str(node.node_id), widths["node"]), _align(label, widths["joint"], left=True)]
    if heartbeat is None:
        return [*row, ("bad", "silent")]
    row.append(_state(heartbeat, now, max_age["heartbeat"], widths["state"]))
    row.append(_cell(node.encoder, now, max_age["encoder"], lambda e: f"{e.pos:.3f}", widths["pos t"]))
    row.append(_cell(node.encoder, now, max_age["encoder"], lambda e: f"{e.vel:.2f}", widths["vel t/s"]))
    row.append(_cell(node.iq, now, max_age["iq"], lambda i: f"{i.measured:.1f}", widths["Iq A"]))
    row.append(_cell(node.torques, now, max_age["torques"], lambda t: f"{t.estimate:.2f}", widths["torque Nm"]))
    row.append(_cell(node.bus_power, now, max_age["bus_power"], lambda b: f"{b.voltage:.1f}", widths["bus V"]))
    row.append(_cell(node.temperature, now, max_age["temperature"], lambda t: f"{t.fet:.0f}", widths["FET C"]))
    row.append(_cell(node.powers, now, max_age["powers"], lambda p: f"{p.electrical:.1f}", widths["power W"]))
    row.append(_age(node, now, max_age["encoder"], widths["age ms"]))
    errors = heartbeat.value.axis_error
    row.append(("bad", describe_errors(errors)) if errors else "")
    return row


def _state(heartbeat: Stamped[Heartbeat], now: float, max_age: float, width: int) -> Segment:
    """The axis state, or how long the node has been silent: a dead node must not look alive."""
    age = heartbeat.age(now)
    if age > max_age:
        return ("bad", _align(f"silent {age:.1f}s", width, left=True))
    state = heartbeat.value.axis_state
    return _align(_STATE_LABELS.get(state, state.name.lower()), width, left=True)


def _cell(stamped: Stamped[T] | None, now: float, max_age: float, render: Callable[[T], str], width: int) -> Segment:
    if stamped is None:
        return ("dim", _align("-", width))
    return _fresh(stamped, now, max_age, _align(render(stamped.value), width))


def _age(node: ODriveNode, now: float, max_age: float, width: int) -> Segment:
    if node.encoder is None:
        return ("dim", _align("-", width))
    return _fresh(node.encoder, now, max_age, _align(f"{node.encoder.age(now) * 1000:.0f}", width))


def _fresh(stamped: Stamped[T], now: float, max_age: float, text: str) -> Segment:
    return ("warn", text) if stamped.age(now) > max_age else text


def _number(value: float, spec: str, width: int) -> str:
    return _align(format(value, spec) if math.isfinite(value) else "-", width)


def _align(text: str, width: int, left: bool = False) -> str:
    """Fixed-width cell with at least one space of separation."""
    text = text[: width - 1]
    return f"{text:<{width}}" if left else f"{text:>{width - 1}} "


def _level_attr(level: int) -> str:
    if level >= logging.ERROR:
        return "bad"
    return "warn" if level >= logging.WARNING else "dim"
