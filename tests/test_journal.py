from pathlib import Path

from darm.arm.journal import Journal


def test_nothing_recorded_reads_as_unknown(tmp_path: Path) -> None:
    assert Journal(tmp_path / "missing" / "journal.json").was_docked() is None


def test_records_survive_a_new_process(tmp_path: Path) -> None:
    Journal(tmp_path / "j.json").mark_away()
    assert Journal(tmp_path / "j.json").was_docked() is False
    Journal(tmp_path / "j.json").mark_docked()
    assert Journal(tmp_path / "j.json").was_docked() is True


def test_a_corrupt_note_reads_as_unknown_not_as_docked(tmp_path: Path) -> None:
    (tmp_path / "j.json").write_text('{"docked": "yes"', encoding="utf-8")
    assert Journal(tmp_path / "j.json").was_docked() is None
