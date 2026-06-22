"""
joystick - Desk-test AUX input from a USB HID joystick (no flight controller).

When the transmitter is plugged into the PC in USB "game controller" mode it
shows up as /dev/input/jsX, not as an FC serial port. Kenet normally reads the
AUX switch from the FC over MSP, so without an FC there is nothing to drive the
IDLE/AI-ARMED/TRACKING state machine.

This module reads the joystick directly and exposes ``JoystickMSP`` — a drop-in
replacement for ``MSPConnection`` that maps joystick switches onto RC channels.
All unmapped channels sit at center and ``send_rc`` is a no-op (there is no FC
to receive RC). This lets you desk-test the state machine + camera + tracker
with just the transmitter.

Find your switch axis with::

    python -m kenet.joystick

Then run::

    python kenet.py --joystick --aux-ch 5 --joy-axis 6 --joy-arm-axis 4 \
        --camera test-2.mp4 --no-gcs

Only stdlib is imported at module load, so the discovery tool above runs with no
third-party packages installed.
"""

import os
import errno
import struct
import logging

logger = logging.getLogger(__name__)

# Linux joystick API: struct js_event { __u32 time; __s16 value; __u8 type; __u8 number; }
_JS_EVENT_FORMAT = "<IhBB"
_JS_EVENT_SIZE = struct.calcsize(_JS_EVENT_FORMAT)  # 8 bytes
_JS_EVENT_BUTTON = 0x01
_JS_EVENT_AXIS = 0x02
_JS_EVENT_INIT = 0x80   # OR'd into type for synthetic initial-state events

_AXIS_MAX = 32767.0     # joystick axis range is [-32767, 32767]


class JoystickInput:
    """Non-blocking reader for a Linux /dev/input/jsX device."""

    def __init__(self, device="/dev/input/js0"):
        self.device = device
        self._fd = None
        self.axes = {}      # number -> latest value (-32767..32767)
        self.buttons = {}   # number -> latest value (0/1)

    def open(self):
        # O_NONBLOCK so poll() never blocks the control loop.
        self._fd = os.open(self.device, os.O_RDONLY | os.O_NONBLOCK)
        logger.info("Joystick opened: %s", self.device)

    def close(self):
        if self._fd is not None:
            os.close(self._fd)
            self._fd = None

    def poll(self):
        """Drain all pending events and update axis/button state.

        Returns the number of events consumed. On open the kernel emits one
        INIT event per axis/button, so the current switch position is known
        after the first poll even if nothing is moved.
        """
        if self._fd is None:
            return 0
        count = 0
        while True:
            try:
                data = os.read(self._fd, _JS_EVENT_SIZE)
            except BlockingIOError:
                break
            except OSError as e:
                if e.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                    break
                raise
            if not data or len(data) < _JS_EVENT_SIZE:
                break
            _t, value, etype, number = struct.unpack(_JS_EVENT_FORMAT, data)
            etype &= ~_JS_EVENT_INIT
            if etype == _JS_EVENT_AXIS:
                self.axes[number] = value
            elif etype == _JS_EVENT_BUTTON:
                self.buttons[number] = value
            count += 1
        return count

    def axis(self, number, default=0):
        return self.axes.get(number, default)


class JoystickMSP:
    """MSPConnection-compatible adapter that sources RC from a USB joystick.

    ``aux_maps`` can map multiple joystick axes onto RC channels. The important
    desk-test case is:
        axis 4 two-state   -> CH5/AUX1 arm-style switch
        axis 5 three-state -> CH7/AUX3 autopilot mode switch
        axis 6 three-state -> CH6/AUX2 or the configured pipeline AUX channel

    There is no FC, so ``send_rc`` and ``get_attitude`` are inert.
    """

    def __init__(self, aux_ch=7, num_channels=8, device="/dev/input/js0",
                 aux_axis=0, center=1500, span=500, invert=False,
                 aux_maps=None):
        self.joystick = JoystickInput(device)
        self.aux_ch = aux_ch
        self.num_channels = num_channels
        self.aux_axis = aux_axis
        self.center = center
        self.span = span
        self.invert = invert
        self.aux_maps = aux_maps

    # ── MSPConnection interface ───────────────────────────────────
    def connect(self):
        try:
            self.joystick.open()
        except OSError as e:
            # Reuse the pipeline's serial-failure path -> graceful preview-only.
            import serial
            raise serial.SerialException(
                "joystick open failed for %s: %s" % (self.joystick.device, e)
            ) from e

    def disconnect(self):
        self.joystick.close()

    def get_rc_channels(self):
        self.joystick.poll()
        rc = [self.center] * self.num_channels
        if self.aux_maps:
            for mapping in self.aux_maps:
                ch = mapping.get("channel")
                axis = mapping.get("axis")
                if ch is None or axis is None or not 0 <= ch < len(rc):
                    continue
                rc[ch] = self._axis_value(
                    axis,
                    mode=mapping.get("mode", "proportional"),
                    invert=mapping.get("invert", False),
                )
        elif 0 <= self.aux_ch < len(rc):
            rc[self.aux_ch] = self._aux_value()
        return rc

    def get_attitude(self):
        return None  # no FC

    def send_rc(self, channels):
        pass  # no FC to receive RC

    def request(self, code):
        return None

    # ── mapping ───────────────────────────────────────────────────
    def _aux_value(self):
        return self._axis_value(
            self.aux_axis,
            mode="proportional",
            invert=self.invert,
        )

    def _axis_value(self, axis, mode="proportional", invert=False):
        raw = self.joystick.axis(axis)
        if invert:
            raw = -raw
        if mode == "two":
            return 2000 if raw > 0 else 1000
        if mode == "three":
            if raw < -_AXIS_MAX / 3:
                return 1000
            if raw > _AXIS_MAX / 3:
                return 2000
            return 1500
        val = self.center + (raw / _AXIS_MAX) * self.span
        return int(max(1000, min(2000, val)))


def main():
    """Live-print joystick axes/buttons so you can identify your AUX switch."""
    import argparse
    import time

    ap = argparse.ArgumentParser(
        description="List live joystick axes/buttons to find your 3-position "
                    "AUX switch. Move the switch and watch which AXIS jumps "
                    "between three values.")
    ap.add_argument("--device", default="/dev/input/js0")
    args = ap.parse_args()

    js = JoystickInput(args.device)
    try:
        js.open()
    except OSError as e:
        raise SystemExit("Cannot open %s: %s" % (args.device, e))

    print("Reading %s — flip your AUX switch through all 3 positions." % args.device)
    print("Note the AXIS number and its low/mid/high values. Ctrl-C to quit.\n")
    try:
        while True:
            js.poll()
            axes = "  ".join("a%d:%+6d" % (k, v) for k, v in sorted(js.axes.items()))
            btns = " ".join("b%d:%d" % (k, v) for k, v in sorted(js.buttons.items()))
            print("\raxes: %s   buttons: %s    " % (axes, btns), end="", flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print()
    finally:
        js.close()


if __name__ == "__main__":
    main()
