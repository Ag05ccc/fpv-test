"""Shared safety helpers for external X11 window candidates."""

from __future__ import annotations

import re
from typing import Any


TOOLING_WINDOW_MARKERS = (
    "visual studio code",
    "vscode",
    '"code"',
    "jetbrains",
    "pycharm",
    "intellij",
    "gnome-terminal",
    "konsole",
    "terminal",
    "xterm",
    "alacritty",
    "kitty",
)
SHELL_TITLE_PATTERN = re.compile(r"^[A-Za-z0-9._-]+@[A-Za-z0-9._-]+:\s")


def tooling_window_rejection_reason(window: dict[str, Any] | None) -> str | None:
    if not isinstance(window, dict):
        return None
    title = str(window.get("title", "")).lower()
    class_text = str(window.get("class_text", "")).lower()
    text = "%s %s" % (title, class_text)
    if any(marker in text for marker in TOOLING_WINDOW_MARKERS):
        return "candidate_window_is_tooling:%s" % window.get("window_id", "unknown")
    if SHELL_TITLE_PATTERN.match(str(window.get("title", "")).strip()):
        return "candidate_window_is_tooling:%s" % window.get("window_id", "unknown")
    return None
