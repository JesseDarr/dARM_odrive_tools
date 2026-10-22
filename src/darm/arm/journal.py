"""Remembers whether the arm was last left in the dock.

ODrives count position from wherever they powered up. Power up in the dock and
zero is the dock. Power up anywhere else and zero, every limit, and the dock
itself silently move with it. The ODrives cannot tell the difference, so the
host keeps a note: 'away' before the arm is first armed, 'docked' only once it
is verifiably back. An arm that reads all zeros but was last left away has
almost certainly been power-cycled out of the dock.
"""

import json
import os
import time
from pathlib import Path


class Journal:
    def __init__(self, path: Path) -> None:
        self.path = path

    def was_docked(self) -> bool | None:
        """True or False as last recorded, None if nothing has been recorded (or the note is unreadable)."""
        try:
            docked = json.loads(self.path.read_text(encoding="utf-8"))["docked"]
        except OSError, ValueError, KeyError, TypeError:
            return None
        return docked if isinstance(docked, bool) else None

    def mark_docked(self) -> None:
        self._record(docked=True)

    def mark_away(self) -> None:
        self._record(docked=False)

    def _record(self, docked: bool) -> None:
        """Write-then-rename, so a power cut mid-write leaves the old note rather than half a new one."""
        self.path.parent.mkdir(parents=True, exist_ok=True)
        scratch = self.path.with_suffix(".tmp")
        scratch.write_text(json.dumps({"docked": docked, "at": time.strftime("%Y-%m-%dT%H:%M:%S")}), encoding="utf-8")
        os.replace(scratch, self.path)
