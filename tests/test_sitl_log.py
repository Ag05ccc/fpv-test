import json
import sys
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parents[1] / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from sitl_log import JsonlLogger  # noqa: E402


def read_events(path):
    return [json.loads(line)["event"] for line in path.read_text().splitlines()]


def test_jsonl_logger_overwrites_by_default(tmp_path):
    path = tmp_path / "run.jsonl"
    with JsonlLogger(path) as logger:
        logger.write("first")

    with JsonlLogger(path) as logger:
        logger.write("second")

    assert read_events(path) == ["second", "session_end"]


def test_jsonl_logger_can_append_when_requested(tmp_path):
    path = tmp_path / "run.jsonl"
    with JsonlLogger(path) as logger:
        logger.write("first")

    with JsonlLogger(path, append=True) as logger:
        logger.write("second")

    assert read_events(path) == ["first", "session_end", "second", "session_end"]


def test_jsonl_logger_batches_flush_but_close_persists_records(tmp_path):
    path = tmp_path / "run.jsonl"
    with JsonlLogger(path, flush_every=10, flush_interval=0.0) as logger:
        logger.write("first")
        logger.write("second")

    assert read_events(path) == ["first", "second", "session_end"]


def test_jsonl_logger_rotates_when_size_limit_is_reached(tmp_path):
    path = tmp_path / "run.jsonl"
    with JsonlLogger(path, metadata={"tool": "test"}, max_bytes=220) as logger:
        for index in range(5):
            logger.write("sample", payload="x" * 80, index=index)

    rotated = tmp_path / "run.001.jsonl"
    assert path.exists()
    assert rotated.exists()
    assert str(rotated) in [str(item) for item in logger.paths]
    assert "sample" in read_events(rotated)
