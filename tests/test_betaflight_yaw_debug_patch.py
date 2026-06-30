import sys
from pathlib import Path


TOOLS_DIR = Path(__file__).resolve().parents[1] / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from betaflight_yaw_debug_patch import (  # noqa: E402
    INSTRUMENTED_SNIPPET,
    TARGET_SNIPPET,
    apply_patch_text,
    patch_state,
    revert_patch_text,
)


def test_patch_state_detects_clean_and_applied_text():
    assert patch_state("prefix\n" + TARGET_SNIPPET + "suffix\n") == "clean"
    assert patch_state("prefix\n" + INSTRUMENTED_SNIPPET + "suffix\n") == "applied"
    assert patch_state("no matching snippet") == "unknown"


def test_apply_patch_text_inserts_instrumentation_once():
    text = "prefix\n" + TARGET_SNIPPET + "suffix\n"

    patched, changed = apply_patch_text(text)
    patched_again, changed_again = apply_patch_text(patched)

    assert changed is True
    assert patch_state(patched) == "applied"
    assert "debug[0]=setpoint" in patched
    assert patched_again == patched
    assert changed_again is False


def test_revert_patch_text_removes_instrumentation_once():
    patched = "prefix\n" + INSTRUMENTED_SNIPPET + "suffix\n"

    clean, changed = revert_patch_text(patched)
    clean_again, changed_again = revert_patch_text(clean)

    assert changed is True
    assert patch_state(clean) == "clean"
    assert "KENET_SITL_YAW_DEBUG_BEGIN" not in clean
    assert clean_again == clean
    assert changed_again is False


def test_apply_patch_text_rejects_unknown_layout():
    try:
        apply_patch_text("unknown")
    except RuntimeError as exc:
        assert "target snippet not found" in str(exc)
    else:
        raise AssertionError("unknown layout should fail")
