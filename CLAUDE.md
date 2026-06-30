# Kenet

Kenet is a visual target-tracking system for FPV drones. A camera feeds an
OpenCV tracker, a PID controller turns the target's position into yaw and pitch
commands, and those commands are sent to a Betaflight flight controller over MSP
(serial). It also sends telemetry to a ground station over UDP. The whole thing
runs on a companion board (Raspberry Pi / Jetson) next to the flight controller,
but it can be tested entirely on a PC.

The state machine has three positions, driven by one 3-position switch:
idle (the pilot flies normally), armed (camera ready, still pilot control), and
tracking (Kenet takes over yaw and pitch through MSP Override).

Current SITL debugging status:

- The physical transmitter/joystick is intentionally isolated out of the Gazebo
  flip investigation for now. When it is reintroduced, run the no-send bench
  preflight first: `tools/sitl_physical_rc_preflight.py --device /dev/input/js0`
  verifies CH5/AUX1 ARM, CH6/AUX2 Kenet state, and CH7/AUX3 mode separation;
  use `--force-mode-pwm 1500` if CH7 is intentionally forced to ANGLE.
- Use the virtual RC path first: UDP 9004 RC packets into Betaflight SITL, with
  CH3 throttle, CH5 ARM, CH6 Kenet state, and CH7 flight mode scripted by
  `tools/sitl_virtual_rc.py` or orchestrated by
  `tools/sitl_virtual_takeoff_check.py`.
- Gazebo motor mapping and FDM yaw sign have been isolated. The active suspect is
  now Betaflight yaw PID/rate feedback behavior under small RC yaw/pitch offsets,
  not the physical RC path and not Kenet's visual tracker.
- The most useful acceptance/debug command family is the runner plus JSONL logs:
  `tools/sitl_virtual_takeoff_check.py --capture-motor-udp --nudge-yaw 1504`
  with optional `--zero-yaw-pid` or `--yaw-pid P,I,D`, then
  `tools/sitl_pid_sweep_summary.py` for diagnostics + raw motor UDP timing.
  Diagnostics/dashboard samples now include Betaflight `MSP_ADVANCED_CONFIG`
  and `MSP_DEBUG` fields as `msp.advanced_config`, `msp.debug_mode`, and
  `msp.debug`. Use `--debug-mode PIDLOOP`, `--debug-mode ANGLERATE`, or
  `--debug-mode ANGLE_TARGET` on the takeoff runner to set Betaflight
  `debug_mode` before the same logged run. When the runner owns Betaflight it
  saves and restarts after setting `debug_mode`, because Betaflight's runtime
  `debugMode` is loaded at init. `sitl_debug_calibration.py` now proves the
  debug path before full Gazebo runs: PIDLOOP is readable but only loop timing,
  ANGLERATE stayed zero, and ANGLE_TARGET did not expose a useful yaw setpoint
  in the P23/yaw1504 flip run. Do not treat MSP_DEBUG as solved yaw-internal
  telemetry yet.
- A marker-based Betaflight instrumentation patch is now available through
  `tools/betaflight_yaw_debug_patch.py`. It reuses `DEBUG_AC_ERROR` to expose
  yaw setpoint, gyroRate, errorRate, P, I, F, S, and Sum. The instrumented P23 /
  yaw1504 run showed setpoint near `-1`, gyro/error around `+610/-610`, and
  yaw P/Sum around `-450` during the flip. The matching P21 / yaw1504
  instrumented run now passes: active-flight debug stays near setpoint/error
  `1`, gyro `0`, and P/Sum `0`, with raw motor spread `0.802`.
- P22 / yaw1504 is the measured boundary: one instrumented run failed with
  active gyro/error `1740/1741`, P/Sum `1243`, raw split at `2.776 s`, and
  attitude fail at `7.668 s`; the repeat passed with active gyro `0`, error
  `1`, P/Sum `0`, and raw spread `0.843`.
- Narrow P23 fix probes did not pass: `--iris-yaw-gyro-scale 0.5/0.25`
  reduced yaw P/Sum but shifted failure toward pitch/attitude,
  `--iris-rotor-vel-p-gain 0.01` delayed but did not remove the split, and
  `--max-step-size 0.001` still failed.
- The first practical P23/yaw1504 fix gate is measured now: Betaflight
  `yaw_rc_rate=5`, `yaw_rate=30`, `yaw_rate_limit=120` made the same
  instrumented P23/yaw1504 run pass twice. The takeoff runner exposes this
  explicit profile as `--safe-yaw-authority`; do not silently treat it as a
  real-flight tune.
- Real video target-found has a neutral acceptance gate now:
  `tools/sitl_video_tracking_check.py --camera test-2.mp4` runs the external
  Gazebo checker plus virtual-pilot `kenet_sitl_mixer.py` on a real video. The
  2026-06-30 PASS used zero pitch/yaw limits: `265` target-found/source=kenet
  mixer samples, max pitch/yaw delta `0/0`, altitude gain `23.850 m`, max
  roll/pitch `0.000/0.100`. This proves the real video tracker path, not
  nonzero command-response tuning. The first nonzero video gate also has a
  measured PASS now, but only when TRACKING is delayed until after takeoff:
  `--yaw-limit 4 --forward-limit 0 --yaw-pid 23,0,0 --kenet-delay-seconds 18`
  gave max yaw delta `4`, altitude gain `31.401 m`, max roll/pitch
  `1.300/5.400`, raw spread `0.000`. The same profile at `--yaw-limit 5`
  also passed with altitude gain `31.377 m`, max roll/pitch `3.300/4.300`,
  raw spread `0.000`, and max yaw delta `5`. The same delayed/P23 profile at
  `--yaw-limit 6` failed with max roll `180.0`, max pitch `72.4`, MSP motor
  spread `945`, and max yaw delta `6`; `--yaw-limit 8` also failed.
  Gecikmesiz yaw4/yaw2/yaw1 video command-response runs failed during
  takeoff/ramp, so do not start real video TRACKING from boot. Current video yaw
  bracket: yaw5 PASS, yaw6 FAIL. Pitch-only video command-response failed with
  the default pitch PID even at `--forward-limit 1`, but passed after adding
  `--pitch-pid 23,0,0`: pitch1/pitch2/pitch4/pitch5 PASS, pitch6 FAIL. Current
  pitch bracket: pitch5 PASS, pitch6 FAIL. Combined pitch+yaw is narrower:
  yaw2+pitch2 and yaw3+pitch3 PASS, yaw4+pitch4 FAIL with the same delayed/P23
  and pitch PID profile.
- Latest measured yaw PID boundaries in this setup: P-only `19,0,0` is stable
  PASS, and clean temp-cwd `21,0,0` repeated PASS (`2 PASS / 0 FAIL`). Earlier
  repo-root cwd runs made `20,0,0` and `22,0,0` look flaky/boundary, and clean
  temp-cwd repeats now show `22,0,0` is still a boundary point (`3 PASS / 1
  FAIL`) while `23,0,0` is repeat FAIL (`2 FAIL`) for yaw 1504. The same
  `23,0,0` with yaw 1502 and yaw 1503 plus a 40s hold passes, so this boundary
  is nudge/setpoint scale dependent and currently sits between yaw 1503 and
  yaw 1504 for P23.
  Treat repo-root `eeprom.bin`/persistent FC state as a strong contributor to
  old P20/P22 flakiness, but not the only explanation. I-only
  passes at `0,1,0` and fails at `0,2,0` for yaw 1504. With `0,2,0`, yaw 1502
  passes the 30s hold, while yaw 1503 produces late raw motor split and fails
  with a longer hold. Treat these as current debug evidence, not as flight
  tuning advice.


## Environment

Important: the system `python3` on this machine may belong to a different
project's virtualenv. Do not install packages into it.

This project's environment is `fpv_env`. There is a shell alias `fpv-test` that
cd's into the project and activates it. It has `opencv-contrib-python`, `numpy`,
`pyserial`, and `pytest`. Run Kenet with it:

    fpv_env/bin/python kenet.py ...

README command examples assume `fpv_env` is already active. From a fresh shell,
prefer `fpv_env/bin/python ...` for Python commands.

(An older `.venv` from earlier setup may still be present; `fpv_env` is the
canonical one.)

`opencv-contrib-python` matters because the CSRT/KCF trackers only exist in the
contrib build of OpenCV. Plain `opencv-python` does not have them, and trying to
track without contrib raises a clear `TrackerUnavailableError`.


## How to test on a PC (no flight controller required)

There are two independent ways to test, and they cover different parts of the
system. Use whichever fits what you want to check.

### Option 1 — Joystick mode (your transmitter as a USB game controller)

When you plug the transmitter into the PC in "USB game controller" mode it shows
up as a joystick (`$JOY_DEV`, default `/dev/input/js0`), not as a flight controller. Kenet normally
reads the AUX switch from a flight controller over MSP, so a small adapter lets
it read the switch straight from the joystick instead. This is the quickest way
to exercise the state machine, the camera, and the tracker without any FC.

First find which joystick axis is your 3-position switch. Run this, flip the
switch through all three positions, and note which axis number jumps between
three values (ideally about -32767, 0, +32767):

    python3 -m kenet.joystick

Then run Kenet in joystick mode. There is no webcam attached right now, so use a
video file for the camera:

    fpv_env/bin/python kenet.py --joystick --joy-axis N --camera test-2.mp4 --no-gcs

Flip the switch: low is idle, middle is armed (you'll see the yellow init box),
high is tracking (the tracker starts on a box in the center of the frame). Add
`--joy-invert` if the direction is reversed.

What this does NOT test: the actual MSP send to a flight controller. In joystick
mode there is no FC, so the override commands go nowhere — that's expected.

### Option 2 — Betaflight SITL (a real Betaflight running on the PC)

Betaflight has a SITL ("software in the loop") build that runs as a normal Linux
program and speaks real MSP. This is the way to test the MSP protocol itself,
the MSP Override behavior, and attitude telemetry against an actual Betaflight,
without any hardware. The build tools needed (gcc, make, cmake, git) are already
installed on this machine.

Build and run it:

    git clone https://github.com/betaflight/betaflight && cd betaflight
    make TARGET=SITL
    ./obj/main/betaflight_SITL.elf

SITL exposes each UART as a TCP port. UART1 (MSP) is at `tcp://127.0.0.1:5761`.
You can configure it with Betaflight Configurator by connecting manually to that
address. Settings are saved to `eeprom.bin` next to the binary.
For local SITL tests, prefer the scripted path first:
`tools/sitl_configure_modes.py` writes ARM/MSP Override/ANGLE/HORIZON mode
ranges, and `tools/sitl_mode_status_check.py` verifies active modes and arming
disable flags over MSP without opening Configurator.
Use `tools/sitl_mixer_matrix_check.py` for the no-hardware P6 receiver/mixer
matrix before treating a manual Receiver-tab observation as necessary.

Kenet can now talk to either a serial FC port or Betaflight SITL's TCP MSP
port. Close Configurator first because the SITL MSP TCP port accepts one client
at a time.

Native TCP path:

    fpv_env/bin/python tools/kenet_msp_smoke.py --msp-tcp 127.0.0.1:5761
    fpv_env/bin/python tools/kenet_msp_smoke.py --msp-tcp 127.0.0.1:5761 --set-raw-rc 1500,1600,1000,1400,2000,2000,1500,1500
    fpv_env/bin/python kenet.py --msp-tcp 127.0.0.1:5761 --no-gcs --camera test-2.mp4

The `--set-raw-rc` smoke uses pilot/AETR channel input and validates the
Betaflight `MSP_RC` readback with the shared AETR-to-MSP mapping. This is the
quickest production-MSP gate before running the full Kenet pipeline.

The old socat pseudo-serial route is still useful as a regression fallback:

    socat pty,raw,echo=0,link=/tmp/ttyBF tcp:127.0.0.1:5761 &
    fpv_env/bin/python kenet.py --port /tmp/ttyBF --no-gcs --camera test-2.mp4

One more thing to know: SITL has no real receiver, so nothing feeds the AUX
switch by default. To drive the state machine in SITL you need to inject RC
values (via a small MSP script, a simulator, or the Configurator). Because of
that, the joystick option above is simpler for AUX/state testing, while SITL is
better for verifying the MSP and override side.


## Unit tests

The tests use fakes and need no hardware. They run in well under a second:

    fpv_env/bin/python -m pytest -q

They cover the MSP encoding/parsing and checksum handling, the controller
(confirming it only drives pitch and yaw, leaving roll and throttle centered),
the pipeline's attitude telemetry and the state-independent stop command, the
tracker factory's error handling, the SITL RC/virtual-RC tools, mixer safety
glue, diagnostics, Gazebo probes, and PID sweep summarizer.


## Things that are easy to get wrong

The transmitter plugged into USB is a joystick, not the flight controller. To
read AUX the normal way, the flight controller itself must be connected over USB
(it shows up as `/dev/ttyACM0`), with the transmitter bound to a receiver on it.

For real flight, set `msp_override_channels = 10` in Betaflight, not 15. Kenet
only drives pitch and yaw, so 15 would also freeze the pilot's throttle and roll
at center. 10 lets the pilot keep throttle and roll while Kenet handles the rest.

There is no webcam attached at the moment, so vision tests need a video file
(for example `test-2.mp4`) rather than `--camera 0`.
