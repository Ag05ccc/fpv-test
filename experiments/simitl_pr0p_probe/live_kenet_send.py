#!/usr/bin/env python3
"""Live Kenet YAW closed-loop for pr0p (yaw-only, mux + smoothing).

Creates the virtual "Kenet Game Sandbox" joystick and drives ONLY its yaw axis.
Bind pr0p's YAW channel to this virtual device; keep roll/pitch/throttle on the
TBS. The viewer is a yaw MUX:
  * normally: pass the pilot's TBS yaw through to the virtual device,
  * CH6 == TRACKING (and locked on the center target): Kenet's smoothed yaw
    takes over (low authority + slew-rate limit so it is gentle).
Roll/pitch/throttle are untouched (still the pilot's TBS -> pr0p, direct).

Keys (focus the viewer window): b=toggle YAW BIND pulse (oscillate virtual yaw
so pr0p can bind to it -- do this DISARMED/on the ground), space=manual lock,
r=unlock, i=invert Kenet yaw sign, p=invert pilot-passthrough sign, q=quit.
"""
from __future__ import annotations

import math
import os
import sys
import time

os.environ.setdefault("DISPLAY", ":1")
REPO = "/home/gz/fpv-test"
sys.path[:0] = [os.path.join(REPO, "experiments/game_screen_sandbox"),
                os.path.join(REPO, "tools"), REPO]

import argparse  # noqa: E402
import cv2  # noqa: E402

from capture_window import FFmpegX11RegionFrameSource, resolve_capture_region  # noqa: E402
from screen_tracking_loop import LoopConfig, build_pipeline_config, controller_command  # noqa: E402
from kenet.tracker import ObjectTracker  # noqa: E402
from kenet.controller import FlightController  # noqa: E402
from virtual_input import AxisCommand, UInputAdapter  # noqa: E402
from sitl_rc_bridge import LinuxJoystick, CHANNEL_MAP, make_channels  # noqa: E402


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def resolve_region(title, retries=1):
    last = None
    for _ in range(max(1, retries)):
        try:
            return resolve_capture_region(window_title=title, window_min_width=200,
                                          window_min_height=200)
        except Exception as e:
            last = e
            time.sleep(0.5)
    raise last


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--window-title", default="pr0p")
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--joy-device", default="/dev/input/js0")
    ap.add_argument("--center-w", type=int, default=96)
    ap.add_argument("--center-h", type=int, default=80)
    ap.add_argument("--tracker", default="CSRT")
    ap.add_argument("--win-x", type=int, default=1960)
    ap.add_argument("--win-y", type=int, default=40)
    ap.add_argument("--yaw-authority", type=float, default=0.40,
                    help="scale on Kenet yaw command (gentle)")
    ap.add_argument("--max-yaw", type=float, default=0.30, help="abs cap on virtual yaw")
    ap.add_argument("--yaw-slew", type=float, default=0.03, help="max yaw change per frame")
    ap.add_argument("--invert-kenet-yaw", action="store_true")
    ap.add_argument("--invert-pilot-yaw", action="store_true")
    ap.add_argument("--geom-poll-s", type=float, default=1.0)
    ap.add_argument("--bind-flag", default="/tmp/kenet_bind_yaw.flag",
                    help="while this file exists, step-pulse the virtual yaw for pr0p binding")
    args = ap.parse_args()

    # Virtual device first (so pr0p can enumerate/bind it).
    adapter = UInputAdapter(name="Kenet Game Sandbox")
    adapter.neutral()
    print("virtual joystick 'Kenet Game Sandbox' created (yaw=ABS_RX)", flush=True)

    joy = LinuxJoystick(args.joy_device)
    joy.open()
    print("joystick open: %s" % args.joy_device, flush=True)

    region = resolve_region(args.window_title, retries=30)
    W, H = region["width"], region["height"]
    print("pr0p region %s" % region, flush=True)

    def build_size_state(w, h):
        cfg = LoopConfig(frame_width=w, frame_height=h, enable_pitch=True)
        return cfg, build_pipeline_config(cfg), (w // 2, h // 2), \
            (w // 2 - args.center_w // 2, h // 2 - args.center_h // 2, args.center_w, args.center_h)

    loop_cfg, pcfg, (cx, cy), center_box = build_size_state(W, H)
    st = {"tracker": None, "controller": None, "locked": False}
    invert_kenet = args.invert_kenet_yaw
    invert_pilot = args.invert_pilot_yaw
    bind_pulse = False
    yaw_ctrl = 0.0  # smoothed yaw state

    def do_lock(frame):
        tr = ObjectTracker(args.tracker)
        tr.init(frame, center_box)
        ctrl = FlightController(pcfg)
        ctrl.set_frame_center(W, H)
        st.update(tracker=tr, controller=ctrl, locked=True)
        print("[LOCK] %s" % (center_box,), flush=True)

    def do_unlock():
        if st["locked"]:
            print("[UNLOCK]", flush=True)
        st.update(tracker=None, controller=None, locked=False)

    win = "Kenet YAW closed-loop"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, W, H)
    cv2.moveWindow(win, args.win_x, args.win_y)

    def new_gen(reg):
        return FFmpegX11RegionFrameSource(reg, fps=args.fps).frames(None)

    gen = new_gen(region)
    manual_lock = False
    fps_disp = 0.0
    tprev = time.monotonic()
    last_geom = 0.0
    last_status = 0.0
    t0 = time.monotonic()

    def put(f, txt, row, col=(255, 255, 255)):
        y = 22 + row * 22
        cv2.putText(f, txt, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(f, txt, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 1, cv2.LINE_AA)

    try:
        while True:
            now = time.monotonic()
            if now - last_geom >= args.geom_poll_s:
                last_geom = now
                try:
                    nr = resolve_region(args.window_title, retries=1)
                except Exception:
                    nr = None
                if nr and any(nr[k] != region[k] for k in ("left", "top", "width", "height")):
                    size_changed = nr["width"] != region["width"] or nr["height"] != region["height"]
                    try:
                        gen.close()
                    except Exception:
                        pass
                    region = nr
                    gen = new_gen(region)
                    if size_changed:
                        W, H = region["width"], region["height"]
                        loop_cfg, pcfg, (cx, cy), center_box = build_size_state(W, H)
                        do_unlock()
                        cv2.resizeWindow(win, W, H)

            try:
                item = next(gen)
            except Exception:
                time.sleep(0.2)
                gen = new_gen(region)
                continue
            frame = item.frame.copy()

            # joystick: yaw (ch idx3) + CH6 (idx5)
            ch_yaw = ch6 = None
            try:
                joy.poll(timeout=0)
                ch = make_channels(joy, CHANNEL_MAP)
                ch_yaw, ch6 = int(ch[3]), int(ch[5])
            except Exception:
                pass
            state = "TRACKING" if (ch6 is not None and ch6 >= 1700) else \
                    ("ARMED" if (ch6 is not None and ch6 >= 1300) else
                     ("IDLE" if ch6 is not None else "n/a"))
            want_lock = (ch6 is not None and ch6 >= 1700) or manual_lock

            if want_lock and not st["locked"]:
                do_lock(item.frame)
            elif not want_lock and st["locked"]:
                do_unlock()

            found = False
            kenet_yaw = pitch_show = yaw_err = 0.0
            if st["locked"] and st["tracker"] is not None:
                res = st["tracker"].update(item.frame)
                st["controller"].update(res)
                cmd = controller_command(st["controller"], pcfg, enable_pitch=True,
                                         yaw_scale=loop_cfg.yaw_scale, pitch_scale=loop_cfg.pitch_scale)
                found = res.found
                kenet_yaw = cmd.yaw
                pitch_show = cmd.pitch
                yaw_err = st["controller"].yaw_error
                if res.bbox:
                    x, y, w, h = [int(v) for v in res.bbox]
                    col = (0, 220, 0) if found else (0, 0, 255)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), col, 2)
            else:
                x, y, w, h = center_box
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 220, 220), 2)
            cv2.drawMarker(frame, (cx, cy), (255, 255, 255), cv2.MARKER_CROSS, 14, 1)

            # ---- YAW MUX + smoothing -> virtual device ----
            bind_active = bind_pulse or (args.bind_flag and os.path.exists(args.bind_flag))
            if bind_active:
                # Strong step-and-hold (not a continuous wave): Unity's interactive
                # rebind ignores "noisy" continuously-moving controls, but locks
                # onto a clean, large, sustained deflection. Repeat so a fresh
                # 0 -> +0.9 actuation happens every ~1.5 s while pr0p listens.
                phase = (now - t0) % 1.5
                yaw_out = 0.9 if phase < 1.0 else 0.0
                yaw_ctrl = yaw_out
                src = "BIND-STEP"
            elif st["locked"]:
                raw = -kenet_yaw if invert_kenet else kenet_yaw
                target = clamp(raw * args.yaw_authority, -args.max_yaw, args.max_yaw)
                yaw_ctrl += clamp(target - yaw_ctrl, -args.yaw_slew, args.yaw_slew)
                yaw_out = yaw_ctrl
                src = "KENET"
            else:
                tbs = 0.0 if ch_yaw is None else (ch_yaw - 1500) / 500.0
                if invert_pilot:
                    tbs = -tbs
                yaw_out = clamp(tbs, -1.0, 1.0)
                yaw_ctrl = yaw_out
                src = "PILOT"
            adapter.send(AxisCommand(yaw=clamp(yaw_out, -1, 1), pitch=0.0, roll=0.0, throttle=0.0))

            dt = now - tprev
            tprev = now
            if dt > 0:
                fps_disp = 0.9 * fps_disp + 0.1 * (1.0 / dt)

            banner = "YAW->virtual  src=%s%s" % (src, "  [BINDING]" if bind_active else "")
            put(frame, banner, 0, (0, 180, 255) if src != "KENET" else (0, 220, 0))
            put(frame, "CH6=%s state=%s locked=%s  yaw_out=%+.2f" % (
                ch6 if ch6 is not None else "n/a", state, st["locked"], yaw_out), 1)
            if st["locked"]:
                put(frame, "FOUND=%s yaw_err=%.0fpx  kenet_yaw=%+.2f (pitch %+.2f shown, NOT sent)" % (
                    found, yaw_err, kenet_yaw, pitch_show), 2, (0, 220, 0) if found else (0, 0, 255))
            else:
                put(frame, "Center target, flip CH6 TRACKING to hand yaw to Kenet", 2)
            put(frame, "inv_kenet=%s inv_pilot=%s auth=%.2f  [b-bind i-inv p-invpilot q-quit]" % (
                invert_kenet, invert_pilot, args.yaw_authority), 3, (180, 180, 180))
            put(frame, "fps=%.1f" % fps_disp, 4, (180, 180, 180))

            cv2.imshow(win, frame)
            if now - last_status >= 1.0:
                last_status = now
                print("src=%s state=%s locked=%s found=%s yaw_out=%+.2f kenet_yaw=%+.2f fps=%.1f" % (
                    src, state, st["locked"], found, yaw_out, kenet_yaw, fps_disp), flush=True)

            k = cv2.waitKey(1) & 0xFF
            if k in (ord('q'), 27):
                break
            elif k == ord(' '):
                manual_lock = not manual_lock
            elif k == ord('r'):
                manual_lock = False
            elif k == ord('b'):
                bind_pulse = not bind_pulse
                print("bind_pulse=%s" % bind_pulse, flush=True)
            elif k == ord('i'):
                invert_kenet = not invert_kenet
                print("invert_kenet=%s" % invert_kenet, flush=True)
            elif k == ord('p'):
                invert_pilot = not invert_pilot
                print("invert_pilot=%s" % invert_pilot, flush=True)
            try:
                if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                    break
            except cv2.error:
                break
    except Exception as exc:
        print("VIEWER ERROR: %s: %s" % (type(exc).__name__, exc), flush=True)
    finally:
        try:
            adapter.send(AxisCommand(yaw=0.0, pitch=0.0, roll=0.0, throttle=0.0))
            adapter.neutral()
            adapter.close()
        except Exception:
            pass
        try:
            joy.close()
        except Exception:
            pass
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
