#!/usr/bin/env python3
"""
Read-only RC joystick monitor for Betaflight SITL mapping work.

The monitor reuses tools/sitl_rc_bridge.py for joystick reading and RC channel
mapping, so the values shown here match the values sent by the bridge.
"""

import argparse
import sys
import time
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

try:
    from sitl_rc_bridge import AXIS_MAX, CHANNEL_MAP, LinuxJoystick, make_channels
    from sitl_rc_channels import first_n_channel_labels
except Exception as exc:
    print("Failed to import sitl_rc_bridge.py: %s" % exc, file=sys.stderr)
    raise SystemExit(2)


tk = None
ttk = None

RC_CHANNEL_LABELS = first_n_channel_labels(8)


def load_tkinter(quiet=False):
    global tk, ttk
    try:
        import tkinter as tk_module
        from tkinter import ttk as ttk_module
    except ModuleNotFoundError:
        if not quiet:
            print(
                "tkinter is not installed. On Ubuntu install it with: "
                "sudo apt install python3-tk",
                file=sys.stderr,
            )
        return False
    tk = tk_module
    ttk = ttk_module
    return True


class ValueBar:
    def __init__(self, parent, name, min_value, max_value, center_value=None):
        self.frame = ttk.Frame(parent)
        self.name = name
        self.min_value = min_value
        self.max_value = max_value
        self.center_value = center_value
        self.value = min_value

        self.name_label = ttk.Label(self.frame, text=name, width=12)
        self.value_label = ttk.Label(self.frame, text="", width=8, anchor="e")
        self.canvas = tk.Canvas(
            self.frame,
            height=20,
            highlightthickness=0,
            bd=0,
            bg="#1f2328",
        )

        self.name_label.grid(row=0, column=0, sticky="w", padx=(0, 8))
        self.canvas.grid(row=0, column=1, sticky="ew")
        self.value_label.grid(row=0, column=2, sticky="e", padx=(8, 0))
        self.frame.columnconfigure(1, weight=1)

        self.canvas.bind("<Configure>", lambda _event: self.draw())
        self.set_value(min_value)

    def grid(self, *args, **kwargs):
        return self.frame.grid(*args, **kwargs)

    def set_value(self, value):
        self.value = value
        self.value_label.configure(text=str(int(round(value))))
        self.draw()

    def draw(self):
        width = max(1, self.canvas.winfo_width())
        height = max(1, self.canvas.winfo_height())
        span = self.max_value - self.min_value
        fraction = 0.0 if span == 0 else (self.value - self.min_value) / span
        fraction = max(0.0, min(1.0, fraction))
        fill_width = int(width * fraction)

        self.canvas.delete("all")
        self.canvas.create_rectangle(0, 0, width, height, fill="#2f363d", width=0)
        self.canvas.create_rectangle(0, 0, fill_width, height, fill="#2f81f7", width=0)

        if self.center_value is not None:
            center_fraction = (self.center_value - self.min_value) / span
            center_x = int(width * max(0.0, min(1.0, center_fraction)))
            self.canvas.create_line(center_x, 0, center_x, height, fill="#d0d7de")


class RcMonitorApp:
    def __init__(self, root, joystick, args):
        self.root = root
        self.joystick = joystick
        self.args = args
        self.start_time = time.monotonic()
        self.last_event_time = None
        self.axis_bars = {}
        self.button_cells = {}
        self.channel_bars = []
        self.mapped_labels = []

        self.root.title("Kenet RC Monitor")
        self.root.geometry("820x620")
        self.root.minsize(640, 500)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self._configure_style()
        self._build_ui()
        self._poll()

    def _configure_style(self):
        style = ttk.Style()
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure("TFrame", background="#0d1117")
        style.configure("TLabelframe", background="#0d1117", foreground="#d0d7de")
        style.configure(
            "TLabelframe.Label",
            background="#0d1117",
            foreground="#d0d7de",
            font=("TkDefaultFont", 10, "bold"),
        )
        style.configure("TLabel", background="#0d1117", foreground="#d0d7de")
        style.configure("Status.TLabel", foreground="#7ee787")
        style.configure("Error.TLabel", foreground="#ff7b72")
        style.configure("Muted.TLabel", foreground="#8b949e")

    def _build_ui(self):
        outer = ttk.Frame(self.root, padding=12)
        outer.pack(fill="both", expand=True)
        outer.columnconfigure(0, weight=1)
        outer.rowconfigure(2, weight=1)

        header = ttk.Frame(outer)
        header.grid(row=0, column=0, sticky="ew", pady=(0, 10))
        header.columnconfigure(0, weight=1)

        title = ttk.Label(header, text="Kenet RC Monitor", font=("TkDefaultFont", 15, "bold"))
        title.grid(row=0, column=0, sticky="w")
        self.status_label = ttk.Label(header, style="Status.TLabel")
        self.status_label.grid(row=1, column=0, sticky="w", pady=(4, 0))

        channels = ttk.Labelframe(outer, text="Mapped RC channels", padding=10)
        channels.grid(row=1, column=0, sticky="ew", pady=(0, 10))
        channels.columnconfigure(0, weight=1)

        for idx, name in enumerate(RC_CHANNEL_LABELS):
            bar = ValueBar(channels, "%s CH%d" % (name, idx + 1), 1000, 2000, 1500)
            bar.grid(row=idx, column=0, sticky="ew", pady=2)
            self.channel_bars.append(bar)

        raw = ttk.Frame(outer)
        raw.grid(row=2, column=0, sticky="nsew")
        raw.columnconfigure(0, weight=1)
        raw.columnconfigure(1, weight=1)
        raw.rowconfigure(0, weight=1)

        self.axes_frame = ttk.Labelframe(raw, text="Raw axes", padding=10)
        self.axes_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 5))
        self.axes_frame.columnconfigure(0, weight=1)

        self.buttons_frame = ttk.Labelframe(raw, text="Raw buttons", padding=10)
        self.buttons_frame.grid(row=0, column=1, sticky="nsew", padx=(5, 0))

        mapping = ttk.Labelframe(outer, text="Active mapping", padding=10)
        mapping.grid(row=3, column=0, sticky="ew", pady=(10, 0))
        mapping.columnconfigure(0, weight=1)
        for row, text in enumerate(self._mapping_lines()):
            label = ttk.Label(mapping, text=text, style="Muted.TLabel")
            label.grid(row=row, column=0, sticky="w")
            self.mapped_labels.append(label)

    def _mapping_lines(self):
        lines = []
        for name, cfg in sorted(CHANNEL_MAP.items(), key=lambda item: item[1]["channel"]):
            source = cfg.get("source")
            index = cfg.get("index")
            ch = cfg.get("channel", 0) + 1
            extra = ""
            if cfg.get("invert"):
                extra += " invert"
            if cfg.get("two_pos"):
                extra += " two-pos"
            if cfg.get("three_pos"):
                extra += " three-pos"
            if source == "fixed":
                extra += " value=%s" % cfg.get("value", 1500)
            lines.append("%-10s -> CH%d from %s %s%s" % (name, ch, source, index, extra))
        return lines

    def _ensure_axis_bar(self, index):
        if index in self.axis_bars:
            return
        row = len(self.axis_bars)
        bar = ValueBar(
            self.axes_frame,
            "Axis %d" % index,
            int(-AXIS_MAX),
            int(AXIS_MAX),
            0,
        )
        bar.grid(row=row, column=0, sticky="ew", pady=2)
        self.axis_bars[index] = bar

    def _ensure_button_cell(self, index):
        if index in self.button_cells:
            return
        row = index // 6
        column = index % 6
        cell = tk.Label(
            self.buttons_frame,
            text="B%02d" % index,
            width=5,
            padx=4,
            pady=4,
            bg="#21262d",
            fg="#d0d7de",
            relief="flat",
        )
        cell.grid(row=row, column=column, sticky="ew", padx=2, pady=2)
        self.button_cells[index] = cell

    def _poll(self):
        try:
            events = self.joystick.poll(timeout=0)
            if events:
                self.last_event_time = time.monotonic()

            channels = make_channels(self.joystick, CHANNEL_MAP)
            self._update_channels(channels)
            self._update_raw_inputs()
            self._update_status(len(events))
        except Exception as exc:
            self.status_label.configure(
                text="Error: %s" % exc,
                style="Error.TLabel",
            )
        finally:
            self.root.after(self.args.refresh_ms, self._poll)

    def _update_channels(self, channels):
        for index, bar in enumerate(self.channel_bars):
            bar.set_value(channels[index])

    def _update_raw_inputs(self):
        for index, value in sorted(self.joystick.axes.items()):
            self._ensure_axis_bar(index)
            self.axis_bars[index].set_value(value)

        for index, value in sorted(self.joystick.buttons.items()):
            self._ensure_button_cell(index)
            cell = self.button_cells[index]
            if value:
                cell.configure(bg="#238636", fg="#ffffff", relief="sunken")
            else:
                cell.configure(bg="#21262d", fg="#d0d7de", relief="flat")

    def _update_status(self, event_count):
        elapsed = time.monotonic() - self.start_time
        if self.last_event_time is None:
            last_event = "waiting for init/events"
        else:
            age = time.monotonic() - self.last_event_time
            last_event = "last event %.1fs ago" % age
        self.status_label.configure(
            text="device=%s refresh=%dms events=%d uptime=%.1fs %s"
            % (self.args.device, self.args.refresh_ms, event_count, elapsed, last_event),
            style="Status.TLabel",
        )

    def close(self):
        self.joystick.close()
        self.root.destroy()


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="/dev/input/js0")
    parser.add_argument("--refresh-ms", type=int, default=50)
    parser.add_argument(
        "--mode",
        choices=("auto", "gui", "text"),
        default="auto",
        help="auto uses GUI when tkinter exists, otherwise text mode",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Text mode only: stop after N seconds; 0 means run until Ctrl-C",
    )
    args = parser.parse_args()
    if args.refresh_ms < 20:
        parser.error("--refresh-ms must be at least 20")
    if args.duration < 0:
        parser.error("--duration must not be negative")
    return args


def open_joystick(args):
    joystick = LinuxJoystick(args.device)
    try:
        joystick.open()
    except OSError as exc:
        print("Failed to open %s: %s" % (args.device, exc), file=sys.stderr)
        return None
    return joystick


def run_gui(args):
    joystick = open_joystick(args)
    if joystick is None:
        return 1

    root = tk.Tk()
    RcMonitorApp(root, joystick, args)
    root.mainloop()
    return 0


def make_meter(value, min_value, max_value, width=28):
    span = max_value - min_value
    fraction = 0.0 if span == 0 else (value - min_value) / span
    fraction = max(0.0, min(1.0, fraction))
    filled = int(round(width * fraction))
    return "[" + "#" * filled + "-" * (width - filled) + "]"


def run_text(args):
    joystick = open_joystick(args)
    if joystick is None:
        return 1

    deadline = None if args.duration <= 0 else time.monotonic() + args.duration
    refresh = args.refresh_ms / 1000.0
    print("\033[?25l", end="")
    try:
        while True:
            if deadline is not None and time.monotonic() >= deadline:
                break

            joystick.poll(timeout=0.02)
            channels = make_channels(joystick, CHANNEL_MAP)
            lines = [
                "Kenet RC Monitor (text mode)",
                "device=%s refresh=%dms" % (args.device, args.refresh_ms),
                "",
                "Mapped RC channels",
            ]
            for index, label in enumerate(RC_CHANNEL_LABELS):
                value = channels[index]
                lines.append(
                    "%-13s %4d %s" %
                    ("%s CH%d" % (label, index + 1), value, make_meter(value, 1000, 2000))
                )

            lines.extend(["", "Raw axes"])
            if joystick.axes:
                for index, value in sorted(joystick.axes.items()):
                    lines.append(
                        "Axis %-2d %6d %s" %
                        (index, value, make_meter(value, -AXIS_MAX, AXIS_MAX))
                    )
            else:
                lines.append("waiting for axis events")

            lines.extend(["", "Raw buttons"])
            if joystick.buttons:
                button_chunks = []
                for index, value in sorted(joystick.buttons.items()):
                    button_chunks.append("[%02d]" % index if value else " %02d " % index)
                    if len(button_chunks) == 8:
                        lines.append(" ".join(button_chunks))
                        button_chunks = []
                if button_chunks:
                    lines.append(" ".join(button_chunks))
            else:
                lines.append("waiting for button events")

            sys.stdout.write("\033[H\033[J" + "\n".join(lines) + "\n")
            sys.stdout.flush()
            time.sleep(refresh)
    except KeyboardInterrupt:
        pass
    finally:
        joystick.close()
        print("\033[?25h", end="")
    return 0


def main():
    args = parse_args()
    if args.mode == "text":
        return run_text(args)

    if args.mode == "gui":
        if not load_tkinter():
            return 2
        return run_gui(args)

    if load_tkinter(quiet=True):
        return run_gui(args)

    print(
        "tkinter is not installed; falling back to text mode. "
        "Install python3-tk for the GUI window.",
        file=sys.stderr,
    )
    return run_text(args)


if __name__ == "__main__":
    raise SystemExit(main())
