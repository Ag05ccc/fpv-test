#!/usr/bin/env python3
"""Live Kenet dry-run viewer for pr0p (window-following capture).

Realtime-captures the pr0p FPV window, draws a center AIM box, and when the
CH6 3-position switch reaches TRACKING (read from the TBS joystick, independent
of pr0p focus) it locks CSRT on whatever is in the center box and shows the
yaw/pitch commands Kenet WOULD send. It NEVER sends control (pure dry-run).

Capture follows the pr0p window: every ~1s the window geometry is re-resolved
(xwininfo) so you can move pr0p and the capture tracks it. The viewer window
is parked in a corner and does not need focus (so pr0p can stay focused for RC).
Keyboard fallback (when the viewer window is focused): SPACE=lock, r=unlock, q=quit.
"""
from __future__ import annotations

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

try:
    from sitl_rc_bridge import LinuxJoystick, CHANNEL_MAP, make_channels
    HAVE_JOY = True
except Exception as _e:  # pragma: no cover
    HAVE_JOY = False
    print("joystick helper import failed: %s" % _e, flush=True)


def resolve_region(title):
    return resolve_capture_region(window_title=title, window_min_width=200,
                                  window_min_height=200)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--window-title", default="pr0p")
    ap.add_argument("--fps", type=float, default=15.0)
    ap.add_argument("--joy-device", default="/dev/input/js0")
    ap.add_argument("--center-w", type=int, default=170)
    ap.add_argument("--center-h", type=int, default=140)
    ap.add_argument("--tracker", default="CSRT")
    ap.add_argument("--win-x", type=int, default=0, help="viewer window x (park it away from pr0p)")
    ap.add_argument("--win-y", type=int, default=0)
    ap.add_argument("--geom-poll-s", type=float, default=1.0)
    args = ap.parse_args()

    region = resolve_region(args.window_title)
    W, H = region["width"], region["height"]
    print("pr0p region %s" % region, flush=True)

    joy = None
    if HAVE_JOY:
        try:
            joy = LinuxJoystick(args.joy_device)
            joy.open()
            print("joystick open: %s (CH6 -> lock trigger)" % args.joy_device, flush=True)
        except Exception as exc:
            print("joystick unavailable (%s); use SPACE" % exc, flush=True)
            joy = None

    # State that depends on frame size.
    def build_size_state(w, h):
        cfg = LoopConfig(frame_width=w, frame_height=h, enable_pitch=True)
        pcfg = build_pipeline_config(cfg)
        cx, cy = w // 2, h // 2
        bw, bh = args.center_w, args.center_h
        box = (cx - bw // 2, cy - bh // 2, bw, bh)
        return cfg, pcfg, (cx, cy), box

    loop_cfg, pcfg, (cx, cy), center_box = build_size_state(W, H)

    st = {"tracker": None, "controller": None, "locked": False}

    def do_lock(frame):
        tr = ObjectTracker(args.tracker)
        tr.init(frame, center_box)
        ctrl = FlightController(pcfg)
        ctrl.set_frame_center(W, H)
        st["tracker"] = tr
        st["controller"] = ctrl
        st["locked"] = True
        print("[LOCK] center box %s" % (center_box,), flush=True)

    def do_unlock():
        if st["locked"]:
            print("[UNLOCK]", flush=True)
        st["tracker"] = None
        st["controller"] = None
        st["locked"] = False

    win = "Kenet Live (dry-run)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, W, H)
    cv2.moveWindow(win, args.win_x, args.win_y)

    def new_gen(reg):
        return FFmpegX11RegionFrameSource(reg, fps=args.fps).frames(None)

    gen = new_gen(region)
    manual_lock = False
    fps_disp = 0.0
    tprev = time.monotonic()
    last_status = 0.0
    last_geom = 0.0
    last_found = None

    def put(frame, txt, row, col=(255, 255, 255)):
        y = 22 + row * 22
        cv2.putText(frame, txt, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(frame, txt, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 1, cv2.LINE_AA)

    try:
        while True:
            now = time.monotonic()

            # --- window-follow: re-resolve pr0p geometry periodically ---
            if now - last_geom >= args.geom_poll_s:
                last_geom = now
                try:
                    nr = resolve_region(args.window_title)
                except Exception:
                    nr = None
                if nr and (nr["left"] != region["left"] or nr["top"] != region["top"]
                           or nr["width"] != region["width"] or nr["height"] != region["height"]):
                    size_changed = (nr["width"] != region["width"] or nr["height"] != region["height"])
                    print("region moved -> %s%s" % (nr, " (size changed)" if size_changed else ""), flush=True)
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
            except StopIteration:
                gen = new_gen(region)
                continue
            except Exception as exc:
                print("capture error: %s; retrying" % exc, flush=True)
                time.sleep(0.3)
                gen = new_gen(region)
                continue

            frame = item.frame.copy()

            ch6 = None
            state = "n/a"
            if joy is not None:
                try:
                    joy.poll(timeout=0)
                    ch = make_channels(joy, CHANNEL_MAP)
                    ch6 = int(ch[5])
                except Exception:
                    ch6 = None
            if ch6 is not None:
                state = "TRACKING" if ch6 >= 1700 else ("ARMED" if ch6 >= 1300 else "IDLE")
                want_lock = (ch6 >= 1700) or manual_lock
            else:
                state = "TRACKING" if manual_lock else "ARMED"
                want_lock = manual_lock

            if want_lock and not st["locked"]:
                do_lock(item.frame)
            elif not want_lock and st["locked"]:
                do_unlock()

            found = False
            yaw_cmd = pitch_cmd = yaw_out = fwd_out = yaw_err = 0.0
            if st["locked"] and st["tracker"] is not None:
                res = st["tracker"].update(item.frame)
                ctrl = st["controller"]
                ctrl.update(res)
                cmd = controller_command(ctrl, pcfg, enable_pitch=True,
                                         yaw_scale=loop_cfg.yaw_scale,
                                         pitch_scale=loop_cfg.pitch_scale)
                found = res.found
                yaw_out, fwd_out, yaw_err = ctrl.yaw_output, ctrl.forward_output, ctrl.yaw_error
                yaw_cmd, pitch_cmd = cmd.yaw, cmd.pitch
                if res.bbox:
                    x, y, w, h = [int(v) for v in res.bbox]
                    col = (0, 220, 0) if found else (0, 0, 255)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), col, 2)
                    if res.center:
                        cv2.circle(frame, (int(res.center[0]), int(res.center[1])), 4, col, -1)
                if last_found and not found:
                    print("[TARGET LOST]", flush=True)
                last_found = found
            else:
                x, y, w, h = center_box
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 220, 220), 2)
                last_found = None

            cv2.drawMarker(frame, (cx, cy), (255, 255, 255), cv2.MARKER_CROSS, 14, 1)

            dt = now - tprev
            tprev = now
            if dt > 0:
                fps_disp = 0.9 * fps_disp + 0.1 * (1.0 / dt)

            put(frame, "DRY-RUN: NO control sent", 0, (0, 180, 255))
            put(frame, "CH6=%s  state=%s  locked=%s" % (
                ch6 if ch6 is not None else "n/a", state, st["locked"]), 1)
            if st["locked"]:
                put(frame, "FOUND=%s  yaw_err=%.0f px" % (found, yaw_err), 2,
                    (0, 220, 0) if found else (0, 0, 255))
                put(frame, "Kenet CMD  yaw=%+.2f  pitch=%+.2f  (yaw_out=%+.0f fwd_out=%+.0f)" % (
                    yaw_cmd, pitch_cmd, yaw_out, fwd_out), 3, (0, 220, 220))
            else:
                put(frame, "Center target, flip CH6 to TRACKING (or SPACE) to lock", 2)
            put(frame, "fps=%.1f  [space=lock r=unlock q=quit]" % fps_disp, 4, (180, 180, 180))

            cv2.imshow(win, frame)

            if now - last_status >= 1.0:
                last_status = now
                print("state=%s locked=%s ch6=%s found=%s yaw=%.2f pitch=%.2f fps=%.1f" % (
                    state, st["locked"], ch6, found, yaw_cmd, pitch_cmd, fps_disp), flush=True)

            k = cv2.waitKey(1) & 0xFF
            if k in (ord('q'), 27):
                print("quit key", flush=True)
                break
            elif k == ord(' '):
                manual_lock = not manual_lock
                print("manual_lock=%s" % manual_lock, flush=True)
            elif k == ord('r'):
                manual_lock = False
            try:
                if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                    print("window closed", flush=True)
                    break
            except cv2.error:
                break
    except Exception as exc:
        print("VIEWER ERROR: %s: %s" % (type(exc).__name__, exc), flush=True)
    finally:
        try:
            gen.close()
        except Exception:
            pass
        if joy is not None:
            try:
                joy.close()
            except Exception:
                pass
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
