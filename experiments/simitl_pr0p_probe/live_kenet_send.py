#!/usr/bin/env python3
"""Live Kenet YAW+PITCH closed-loop for pr0p (mux + smoothing, dry pilot fallback).

Creates the virtual "Kenet Game Sandbox" joystick and feeds it every axis:

  * yaw   : MUX -- pilot's TBS yaw normally; Kenet's smoothed yaw when CH6 ==
            TRACKING and locked on the center target.
  * pitch : MUX -- same, for pitch (Kenet's approach/forward command).
  * roll  : always the pilot's TBS roll (passthrough).
  * thr   : always the pilot's TBS throttle (passthrough).

Roll/throttle are passed through as a SAFETY NET: pr0p still has generic
`<Joystick>/Stick/x` style bindings for some channels, which are ambiguous once
two joysticks exist -- by mirroring the pilot's stick onto the virtual device,
the pilot's input reaches pr0p no matter which device a generic binding resolves
to. Bind pr0p's YAW and PITCH to this virtual device; leave the rest as-is.

Bind helper: while the yaw/pitch bind-flag file exists, that axis does a strong
step-and-hold pulse (0 -> +0.9 -> 0) so pr0p's interactive rebind locks onto it.
Unity's rebind ignores continuously-moving ("noisy") controls, which is why this
is a step and not a wave. Bind only while DISARMED / on the ground.

Keys (viewer window focused): space=manual lock, r=unlock, i/o=invert Kenet
yaw/pitch, q=quit.
"""
from __future__ import annotations

import json
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


def resolve_region(title, retries=1, exact=False):
    # exact=True matters: a terminal sitting in .../simitl-pr0p has "pr0p" in its
    # title and would otherwise be captured instead of the game window.
    last = None
    for _ in range(max(1, retries)):
        try:
            return resolve_capture_region(window_title=title, window_exact=exact,
                                          window_min_width=200, window_min_height=200)
        except Exception as e:
            last = e
            time.sleep(0.5)
    raise last


def step_pulse(now, t0):
    """Strong 0 -> +0.9 -> 0 step, repeating every 1.5 s."""
    return 0.9 if ((now - t0) % 1.5) < 1.0 else 0.0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--window-title", default="pr0p")
    ap.add_argument("--window-exact", action="store_true")
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--joy-device", default="/dev/input/js0")
    ap.add_argument("--center-w", type=int, default=96)
    ap.add_argument("--center-h", type=int, default=80)
    ap.add_argument("--tracker", default="CSRT")
    ap.add_argument("--win-x", type=int, default=1960)
    ap.add_argument("--win-y", type=int, default=40)
    ap.add_argument("--yaw-authority", type=float, default=0.40)
    ap.add_argument("--max-yaw", type=float, default=0.30)
    ap.add_argument("--yaw-slew", type=float, default=0.03)
    ap.add_argument("--pitch-authority", type=float, default=0.35)
    ap.add_argument("--max-pitch", type=float, default=0.25)
    ap.add_argument("--pitch-slew", type=float, default=0.025)
    ap.add_argument("--invert-kenet-yaw", action="store_true")
    ap.add_argument("--invert-kenet-pitch", action="store_true")
    ap.add_argument("--invert-pilot-yaw", action="store_true")
    ap.add_argument("--invert-pilot-pitch", action="store_true")
    ap.add_argument("--no-kenet-pitch", action="store_true",
                    help="keep pitch on the pilot even while tracking (yaw-only stage)")
    ap.add_argument("--desired-target-width", type=float, default=260.0,
                    help="Kenet pitches forward until the tracked target is this wide "
                         "(px). Must be well above the lock-box width or the approach "
                         "error is ~0 and Kenet never pitches.")
    ap.add_argument("--geom-poll-s", type=float, default=1.0)
    ap.add_argument("--bind-flag-yaw", default="/tmp/kenet_bind_yaw.flag")
    ap.add_argument("--bind-flag-pitch", default="/tmp/kenet_bind_pitch.flag")
    # Sign flips via flag files: focusing the viewer mid-flight would make pr0p
    # lose focus and stop reading RC, so the sign must be fixable without focus.
    ap.add_argument("--invert-flag-yaw", default="/tmp/kenet_invert_yaw.flag")
    ap.add_argument("--invert-flag-pitch", default="/tmp/kenet_invert_pitch.flag")
    # Live gain tuning without a restart (a restart re-creates the uinput device,
    # which briefly drops the pilot's yaw/pitch -- unsafe mid-flight).
    ap.add_argument("--tune-file", default="/tmp/kenet_tune.json",
                    help="optional JSON: yaw_authority, max_yaw, yaw_slew, "
                         "pitch_authority, max_pitch, pitch_slew")
    args = ap.parse_args()

    adapter = UInputAdapter(name="Kenet Game Sandbox")
    adapter.neutral()
    print("virtual joystick 'Kenet Game Sandbox' created (yaw=ABS_RX pitch=ABS_Y)", flush=True)

    joy = LinuxJoystick(args.joy_device)
    joy.open()
    print("joystick open: %s" % args.joy_device, flush=True)

    print("waiting for the pr0p window (launch pr0p now)...", flush=True)
    region = resolve_region(args.window_title, retries=240, exact=args.window_exact)
    W, H = region["width"], region["height"]
    print("pr0p region %s" % region, flush=True)

    def build_size_state(w, h):
        cfg = LoopConfig(frame_width=w, frame_height=h, enable_pitch=True,
                         desired_target_width=args.desired_target_width)
        return cfg, build_pipeline_config(cfg), (w // 2, h // 2), \
            (w // 2 - args.center_w // 2, h // 2 - args.center_h // 2,
             args.center_w, args.center_h)

    loop_cfg, pcfg, (cx, cy), center_box = build_size_state(W, H)
    st = {"tracker": None, "controller": None, "locked": False}
    inv_ky, inv_kp = args.invert_kenet_yaw, args.invert_kenet_pitch
    yaw_ctrl = pitch_ctrl = 0.0

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

    win = "Kenet closed-loop (yaw+pitch)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, W, H)
    cv2.moveWindow(win, args.win_x, args.win_y)

    def new_gen(reg):
        return FFmpegX11RegionFrameSource(reg, fps=args.fps).frames(None)

    gen = new_gen(region)
    manual_lock = False
    fps_disp = 0.0
    tprev = t0 = time.monotonic()
    last_geom = last_status = last_tune = 0.0

    tune = {"yaw_authority": args.yaw_authority, "max_yaw": args.max_yaw,
            "yaw_slew": args.yaw_slew, "pitch_authority": args.pitch_authority,
            "max_pitch": args.max_pitch, "pitch_slew": args.pitch_slew,
            "center_w": float(args.center_w), "center_h": float(args.center_h)}

    def reload_tune():
        try:
            with open(args.tune_file) as fh:
                data = json.load(fh)
            changed = {k: float(v) for k, v in data.items() if k in tune
                       and float(v) != tune[k]}
            if changed:
                tune.update(changed)
                print("tune updated: %s" % changed, flush=True)
        except Exception:
            pass

    def put(f, txt, row, col=(255, 255, 255)):
        y = 22 + row * 22
        cv2.putText(f, txt, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(f, txt, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 1, cv2.LINE_AA)

    try:
        while True:
            now = time.monotonic()
            if now - last_tune >= 0.5:
                last_tune = now
                reload_tune()
            # Recompute the centre lock box each frame so it can be resized live
            # (takes effect on the next lock).
            _bw, _bh = max(16, int(tune["center_w"])), max(16, int(tune["center_h"]))
            center_box = (cx - _bw // 2, cy - _bh // 2, _bw, _bh)
            if now - last_geom >= args.geom_poll_s:
                last_geom = now
                try:
                    nr = resolve_region(args.window_title, retries=1, exact=args.window_exact)
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

            # ---- pilot sticks from the TBS (js0) ----
            p_roll = p_pitch = p_thr = p_yaw = 0.0
            ch6 = None
            try:
                joy.poll(timeout=0)
                ch = make_channels(joy, CHANNEL_MAP)
                p_roll = (ch[0] - 1500) / 500.0
                p_pitch = (ch[1] - 1500) / 500.0
                p_thr = (ch[2] - 1500) / 500.0
                p_yaw = (ch[3] - 1500) / 500.0
                ch6 = int(ch[5])
            except Exception:
                pass
            if args.invert_pilot_yaw:
                p_yaw = -p_yaw
            if args.invert_pilot_pitch:
                p_pitch = -p_pitch

            state = "TRACKING" if (ch6 is not None and ch6 >= 1700) else \
                    ("ARMED" if (ch6 is not None and ch6 >= 1300) else
                     ("IDLE" if ch6 is not None else "n/a"))
            want_lock = (ch6 is not None and ch6 >= 1700) or manual_lock
            if want_lock and not st["locked"]:
                do_lock(item.frame)
            elif not want_lock and st["locked"]:
                do_unlock()

            found = False
            k_yaw = k_pitch = yaw_err = 0.0
            if st["locked"] and st["tracker"] is not None:
                res = st["tracker"].update(item.frame)
                st["controller"].update(res)
                cmd = controller_command(st["controller"], pcfg, enable_pitch=True,
                                         yaw_scale=loop_cfg.yaw_scale,
                                         pitch_scale=loop_cfg.pitch_scale)
                found = res.found
                k_yaw, k_pitch = cmd.yaw, cmd.pitch
                yaw_err = st["controller"].yaw_error
                if res.bbox:
                    x, y, w, h = [int(v) for v in res.bbox]
                    col = (0, 220, 0) if found else (0, 0, 255)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), col, 2)
            else:
                x, y, w, h = center_box
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 220, 220), 2)
            cv2.drawMarker(frame, (cx, cy), (255, 255, 255), cv2.MARKER_CROSS, 14, 1)

            # ---- YAW mux ----
            bind_y = os.path.exists(args.bind_flag_yaw)
            bind_p = os.path.exists(args.bind_flag_pitch)
            inv_ky = args.invert_kenet_yaw or os.path.exists(args.invert_flag_yaw)
            inv_kp = args.invert_kenet_pitch or os.path.exists(args.invert_flag_pitch)
            if bind_y:
                yaw_out = step_pulse(now, t0)
                yaw_ctrl = yaw_out
                ysrc = "BIND"
            elif st["locked"]:
                tgt = clamp((-k_yaw if inv_ky else k_yaw) * tune["yaw_authority"],
                            -tune["max_yaw"], tune["max_yaw"])
                yaw_ctrl += clamp(tgt - yaw_ctrl, -tune["yaw_slew"], tune["yaw_slew"])
                yaw_out = yaw_ctrl
                ysrc = "KENET"
            else:
                yaw_out = clamp(p_yaw, -1, 1)
                yaw_ctrl = yaw_out
                ysrc = "PILOT"

            # ---- PITCH mux ----
            if bind_p:
                pitch_out = step_pulse(now, t0)
                pitch_ctrl = pitch_out
                psrc = "BIND"
            elif st["locked"] and not args.no_kenet_pitch:
                tgt = clamp((-k_pitch if inv_kp else k_pitch) * tune["pitch_authority"],
                            -tune["max_pitch"], tune["max_pitch"])
                pitch_ctrl += clamp(tgt - pitch_ctrl, -tune["pitch_slew"], tune["pitch_slew"])
                pitch_out = pitch_ctrl
                psrc = "KENET"
            else:
                pitch_out = clamp(p_pitch, -1, 1)
                pitch_ctrl = pitch_out
                psrc = "PILOT"

            # roll/throttle: always mirror the pilot (safety net for generic bindings)
            adapter.send(AxisCommand(yaw=clamp(yaw_out, -1, 1),
                                     pitch=clamp(pitch_out, -1, 1),
                                     roll=clamp(p_roll, -1, 1),
                                     throttle=clamp(p_thr, -1, 1)))

            dt = now - tprev
            tprev = now
            if dt > 0:
                fps_disp = 0.9 * fps_disp + 0.1 * (1.0 / dt)

            col_y = (0, 220, 0) if ysrc == "KENET" else ((0, 0, 255) if ysrc == "BIND" else (0, 180, 255))
            put(frame, "yaw=%s %+.2f   pitch=%s %+.2f   (roll/thr = pilot)" % (
                ysrc, yaw_out, psrc, pitch_out), 0, col_y)
            put(frame, "CH6=%s state=%s locked=%s" % (
                ch6 if ch6 is not None else "n/a", state, st["locked"]), 1)
            if st["locked"]:
                put(frame, "FOUND=%s yaw_err=%.0fpx  kenet yaw=%+.2f pitch=%+.2f" % (
                    found, yaw_err, k_yaw, k_pitch), 2, (0, 220, 0) if found else (0, 0, 255))
            else:
                put(frame, "Center target, flip CH6 TRACKING to hand yaw+pitch to Kenet", 2)
            put(frame, "inv_kenet yaw=%s pitch=%s  [i/o invert, space lock, q quit]" % (
                inv_ky, inv_kp), 3, (180, 180, 180))
            put(frame, "fps=%.1f" % fps_disp, 4, (180, 180, 180))

            cv2.imshow(win, frame)
            if now - last_status >= 1.0:
                last_status = now
                print("yaw=%s%+.2f pitch=%s%+.2f state=%s locked=%s found=%s "
                      "kenet(y=%+.2f p=%+.2f) fps=%.1f" % (
                          ysrc, yaw_out, psrc, pitch_out, state, st["locked"], found,
                          k_yaw, k_pitch, fps_disp), flush=True)

            k = cv2.waitKey(1) & 0xFF
            if k in (ord('q'), 27):
                break
            elif k == ord(' '):
                manual_lock = not manual_lock
            elif k == ord('r'):
                manual_lock = False
            elif k in (ord('i'), ord('o')):
                # Toggle the flag file, which is the single source of truth for
                # the sign (so it can also be flipped from outside, without focus).
                path = args.invert_flag_yaw if k == ord('i') else args.invert_flag_pitch
                if os.path.exists(path):
                    os.remove(path)
                else:
                    open(path, "w").close()
                print("invert flag %s -> %s" % (path, os.path.exists(path)), flush=True)
            try:
                if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                    break
            except cv2.error:
                break
    except Exception as exc:
        print("VIEWER ERROR: %s: %s" % (type(exc).__name__, exc), flush=True)
    finally:
        try:
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
