#!/usr/bin/env python3
"""X11 window discovery helpers for the game/screen sandbox."""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
from dataclasses import dataclass
from typing import Iterable


GEOM_INT_RE = r"(?:\+-\d+|[+-]\d+)"


WINDOW_RE = re.compile(
    r"^\s*"
    r"(?P<window_id>0x[0-9a-fA-F]+)\s+"
    r"(?:(?:\"(?P<title>.*?)\")|\(has no name\)):\s+"
    r"\((?P<class_text>.*?)\)\s+"
    r"(?P<width>\d+)x(?P<height>\d+)"
    r"(?P<rel_x>" + GEOM_INT_RE + r")(?P<rel_y>" + GEOM_INT_RE + r")\s+"
    r"(?P<abs_x>" + GEOM_INT_RE + r")(?P<abs_y>" + GEOM_INT_RE + r")"
)


@dataclass(frozen=True)
class X11Window:
    window_id: str
    title: str
    class_text: str
    width: int
    height: int
    rel_x: int
    rel_y: int
    abs_x: int
    abs_y: int

    @property
    def area(self) -> int:
        return self.width * self.height

    def region(self) -> dict[str, int]:
        return {
            "left": self.abs_x,
            "top": self.abs_y,
            "width": self.width,
            "height": self.height,
        }

    def as_dict(self) -> dict[str, object]:
        return {
            "window_id": self.window_id,
            "title": self.title,
            "class_text": self.class_text,
            "width": self.width,
            "height": self.height,
            "rel_x": self.rel_x,
            "rel_y": self.rel_y,
            "abs_x": self.abs_x,
            "abs_y": self.abs_y,
            "area": self.area,
            "region": self.region(),
        }


def parse_xwininfo_tree(text: str) -> list[X11Window]:
    windows: list[X11Window] = []
    for line in text.splitlines():
        match = WINDOW_RE.match(line)
        if not match:
            continue
        title = match.group("title") or ""
        windows.append(X11Window(
            window_id=match.group("window_id"),
            title=title,
            class_text=match.group("class_text"),
            width=int(match.group("width")),
            height=int(match.group("height")),
            rel_x=parse_geometry_int(match.group("rel_x")),
            rel_y=parse_geometry_int(match.group("rel_y")),
            abs_x=parse_geometry_int(match.group("abs_x")),
            abs_y=parse_geometry_int(match.group("abs_y")),
        ))
    return windows


def parse_geometry_int(value: str) -> int:
    return int(value.replace("+-", "-"))


def list_x11_windows(*, xwininfo_bin: str = "xwininfo") -> list[X11Window]:
    display = os.environ.get("DISPLAY")
    if not display:
        raise RuntimeError("DISPLAY is required for X11 window discovery")
    proc = subprocess.run(
        [xwininfo_bin, "-root", "-tree"],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    return parse_xwininfo_tree(proc.stdout)


def visible_windows(
    windows: Iterable[X11Window],
    *,
    min_width: int = 32,
    min_height: int = 32,
) -> list[X11Window]:
    return [
        window for window in windows
        if window.title
        and window.width >= min_width
        and window.height >= min_height
        and window.abs_x >= 0
        and window.abs_y >= 0
    ]


def find_window_by_title(
    title: str,
    windows: Iterable[X11Window],
    *,
    exact: bool = False,
    case_sensitive: bool = False,
    min_width: int = 32,
    min_height: int = 32,
    preferred_size: tuple[int, int] | None = None,
) -> X11Window | None:
    if not title:
        raise ValueError("title must be non-empty")
    needle = title if case_sensitive else title.lower()
    candidates = visible_windows(windows, min_width=min_width, min_height=min_height)
    matches: list[X11Window] = []
    for window in candidates:
        haystack = window.title if case_sensitive else window.title.lower()
        if (haystack == needle) if exact else (needle in haystack):
            matches.append(window)
    if not matches:
        return None
    if preferred_size is not None:
        preferred_width, preferred_height = preferred_size
        return sorted(
            matches,
            key=lambda window: (
                abs(window.width - preferred_width) + abs(window.height - preferred_height),
                -window.area,
            ),
        )[0]
    return sorted(matches, key=lambda window: window.area, reverse=True)[0]


def resolve_window_region(
    title: str,
    *,
    exact: bool = False,
    case_sensitive: bool = False,
    min_width: int = 32,
    min_height: int = 32,
    preferred_size: tuple[int, int] | None = None,
) -> dict[str, int]:
    window = find_window_by_title(
        title,
        list_x11_windows(),
        exact=exact,
        case_sensitive=case_sensitive,
        min_width=min_width,
        min_height=min_height,
        preferred_size=preferred_size,
    )
    if window is None:
        raise RuntimeError("no visible X11 window matched title: %s" % title)
    return window.region()


def parse_size(value: str) -> tuple[int, int]:
    parts = value.lower().split("x")
    if len(parts) != 2:
        raise argparse.ArgumentTypeError("size must be WIDTHxHEIGHT")
    width, height = [int(part.strip()) for part in parts]
    if width <= 0 or height <= 0:
        raise argparse.ArgumentTypeError("size width/height must be positive")
    return (width, height)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--list", action="store_true")
    parser.add_argument("--title")
    parser.add_argument("--exact", action="store_true")
    parser.add_argument("--case-sensitive", action="store_true")
    parser.add_argument("--min-width", type=int, default=32)
    parser.add_argument("--min-height", type=int, default=32)
    parser.add_argument("--preferred-size", type=parse_size,
                        help="Prefer matching windows closest to WIDTHxHEIGHT")
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args(argv)
    if not args.list and not args.title:
        parser.error("use --list or --title")
    if args.min_width <= 0 or args.min_height <= 0:
        parser.error("--min-width/--min-height must be positive")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    windows = visible_windows(
        list_x11_windows(),
        min_width=args.min_width,
        min_height=args.min_height,
    )
    if args.list:
        rows = [window.as_dict() for window in windows]
        if args.json:
            print(json.dumps(rows, indent=2, sort_keys=True))
        else:
            for window in windows:
                region = window.region()
                print("%s %dx%d+%d+%d %s" % (
                    window.window_id,
                    region["width"],
                    region["height"],
                    region["left"],
                    region["top"],
                    window.title,
                ))
        return 0

    match = find_window_by_title(
        args.title,
        windows,
        exact=args.exact,
        case_sensitive=args.case_sensitive,
        min_width=args.min_width,
        min_height=args.min_height,
        preferred_size=args.preferred_size,
    )
    if match is None:
        print("window WAITING title=%s" % args.title)
        return 2
    if args.json:
        print(json.dumps(match.as_dict(), indent=2, sort_keys=True))
    else:
        region = match.region()
        print("window PASS title=%s region=%d,%d,%d,%d" % (
            match.title,
            region["left"],
            region["top"],
            region["width"],
            region["height"],
        ))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
