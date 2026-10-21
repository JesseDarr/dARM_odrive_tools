"""Gamepad state -> joint velocities. Pure: no pygame, no arm, no clock.

Every rule that decides whether a stick moves the arm lives here, free of I/O,
so each one is testable without a controller in hand: the dead-man, the shift
button, the dead zone, where a released trigger rests, and the speed cap.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence

from darm.config import GamepadBinding, GamepadSpec

TRIGGER_REST = -1.0  # SDL reports a released analogue trigger as -1 and a fully pulled one as +1.


def joint_velocities(
    axes: Sequence[float],
    buttons: Sequence[bool],
    spec: GamepadSpec,
    max_speeds: Mapping[str, float],
) -> dict[str, float]:
    """Joint units/s for every joint the controller is asking to move. Empty unless the dead-man is held.

    Only joints named in max_speeds can move, and never faster than their max speed. A jog
    faster than the arm's own slew limit would leave the target running ahead of the joint,
    and the joint would keep going after the stick was let go.
    """
    if not is_pressed(buttons, spec.deadman_button):
        return {}
    shifted = is_pressed(buttons, spec.shift_button)
    velocities: dict[str, float] = {}
    for binding in spec.bindings:
        if binding.joint in max_speeds and _applies(binding, shifted):
            deflection = apply_dead_zone(_deflection(axes, binding), spec.dead_zone)
            velocities[binding.joint] = velocities.get(binding.joint, 0.0) + deflection * binding.speed
    return {joint: _cap(v, max_speeds[joint]) for joint, v in velocities.items() if v}


def apply_dead_zone(value: float, dead_zone: float) -> float:
    """Zero inside the dead zone, then rescaled so output rises continuously to 1 at full deflection."""
    magnitude = abs(value)
    if magnitude <= dead_zone:
        return 0.0
    scaled = min(1.0, (magnitude - dead_zone) / (1.0 - dead_zone))
    return math.copysign(scaled, value)


def _deflection(axes: Sequence[float], binding: GamepadBinding) -> float:
    """One axis as -1..1, or 0..1 for a trigger. A missing or garbage reading counts as released."""
    if not 0 <= binding.axis < len(axes) or not math.isfinite(axes[binding.axis]):
        return 0.0
    value = max(-1.0, min(1.0, axes[binding.axis]))
    return (value - TRIGGER_REST) / (1.0 - TRIGGER_REST) if binding.trigger else value


def _applies(binding: GamepadBinding, shifted: bool) -> bool:
    return binding.when == "always" or (binding.when == "shifted") == shifted


def is_pressed(buttons: Sequence[bool], index: int) -> bool:
    """A button the controller does not have is never pressed."""
    return 0 <= index < len(buttons) and bool(buttons[index])


def _cap(velocity: float, max_speed: float) -> float:
    return max(-max_speed, min(max_speed, velocity))
