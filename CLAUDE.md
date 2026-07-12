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

START HERE (agents): read `AGENT-STATUS.md` first — it holds the honest current
state (perception + no-command flight work; closed-loop tracking does NOT — the
drone spins/hovers instead of moving toward the target) and the measurement
pitfalls that made earlier "PASS" claims false. Do not trust older "tracking
works"/"A4 PASS" notes below without checking it.

The project roadmap is `roadmap-v2.md` (2026-07-04): requirements for the
moving-target visual-tracking goal, each mapped to a measurable gate, plus the
measured status of all nine core metrics (arm/disarm, prop spin, climb, RPY
response, MSP path, tracking PID, tracker robustness, moving targets, camera
feed). New in that turn, all live-verified: `kenet/gz_camera.py` (Gazebo camera
topic -> tracker bridge, camera source `gz:/kenet/fpv_camera`, measured 30 fps
via `tools/sitl_gz_camera_probe.py`), `tools/sitl_target_mover.py` (set_pose
target driver + ground-truth JSONL, measured 8 m displacement with 0.025 m mean
path error; note its two measured gz-transport pitfalls: lazy service discovery
needs the warm-up step, and hammering after lost replies multiplies reply
loss), `tools/sitl_run_quality_check.py` (automated INVALID/PASS/FAIL verdicts:
run validity, RTF/cadence health, window-normalized climb rate, report-only
disarm check, signed yaw response, 5/5 chain), and offline gates
`tools/tracker_motion_benchmark.py` + `tests/test_controller_convergence.py`.

Current SITL debugging status:

- The physical transmitter/joystick is permanently optional (user decision,
  2026-07-02): all development and testing runs headlessly via virtual RC /
  scripted MSP, and no gate may require the user to operate the physical
  transmitter. If the user chooses to try it on their own initiative, the
  no-send bench preflight runs first:
  `tools/sitl_physical_rc_preflight.py --device /dev/input/js0`
  verifies CH5/AUX1 ARM, CH6/AUX2 Kenet state, and CH7/AUX3 mode separation;
  use `--force-mode-pwm 1500` if CH7 is intentionally forced to ANGLE.
- The virtual RC path is the primary path: UDP 9004 RC packets into Betaflight
  SITL, with CH3 throttle, CH5 ARM, CH6 Kenet state, and CH7 flight mode
  scripted by `tools/sitl_virtual_rc.py` or orchestrated by
  `tools/sitl_virtual_takeoff_check.py`. For human-in-the-loop flying there is
  also `tools/sitl_keyboard_rc.py` (keyboard → UDP 9004, no hardware, safe
  startup/exit) used by `./launch-fpv-sim.sh`.
- FPV layer (2026-07-03): `run_gazebo_betaflight.sh --iris-forward-camera`
  injects a forward camera (topic `/kenet/fpv_camera`, 640x480@30, 1 g link)
  into the temporary Iris model copy — acceptance runs stay camera-free on
  purpose so the render-free evidence base is unchanged. Local scenery models
  `kenet_person`, `kenet_person_blue`, `kenet_car`, `kenet_car_white` and the
  world `betaloop_iris_betaflight_demo_populated.sdf` (world name still
  `betaloop_demo`) live in `~/aeroloop_gazebo`; the populated world is
  measured flight-equivalent (safe-yaw P23/yaw1504 acceptance window PASS on
  it: altitude 22.985, roll/pitch 0.000, spread 0.683). `./launch-fpv-sim.sh`
  starts Gazebo GUI (ImageDisplay FPV panel via `tools/fpv_gui.config`) +
  SITL (temp cwd, modes + safe rates + PID23) + keyboard RC by default,
  `--input joystick` for the physical stick (user initiative only).
  Camera-ON flight requires the truthful-timing profile: the camera's render
  load tips the sync-OFF world into a roll-180 flip (measured 2/2 FAIL vs
  camera-off PASS on the same flow), while sync + all-axis tune flies clean
  (roll 1.0) — so the FPV launcher exports `KENET_SITL_LOOPTIME_US` and
  imports `configs/fpv-sim.txt` into the eeprom before SITL. That config also
  fixes a manual-arming block found 2026-07-03: with only the tune profile
  (small_angle=25 default), the SITL accelerometer can be slow to report a
  clean gravity vector, so Betaflight holds the `ANGLE` arming-disable and
  refuses to arm even on a dead-level drone (motors stay at 0 → props never
  spin — verified live over MSP: acc read 0/0/0 while attitude and Gazebo pose
  were both level). `configs/fpv-sim.txt` adds the SITL essentials
  (`small_angle=180` to bypass the tilt check, `feature -3D`,
  `motor_pwm_protocol=PWM`, `runaway_takeoff_prevention=OFF`) on top of the
  roll P23 tune. Proven: headless run with the full launcher config
  (populated world + camera + sync + fpv-sim.txt) armed (59 ARM+ANGLE
  samples), motors spun, climbed 10.2 m, roll/pitch 0.0/0.2, PASS.
  `sitl_virtual_takeoff_check.py` now has `--iris-forward-camera` so this
  exact launcher config can be replayed headlessly.
- The measurable "SITL is healthy / flight is clean" verdict is defined in
  `docs/sitl-flight-readiness-criteria.md`: layered gates (env, static,
  sim-health metrics, flight-quality metrics, repeatability 5/5, Kenet
  integration), plus run-validity rules — a run with `armed_angle_samples=0`
  or a dirty first diagnostic sample (e.g. roll `-180` carried over from a
  previous run, as measured in the first-round pitch1515/1520/1522 bracket
  runs) is INVALID, not FAIL, and must not be cited as bracket evidence.
- Gazebo motor mapping and FDM yaw sign have been isolated, and the
  timing/lockstep hypothesis is now measured and excluded (2026-07-02): RTF is
  ~1.000 in every examined run (0.9935–1.0013), motor UDP cadence is
  metronomic and statistically identical in passing and failing runs, no
  packet gap precedes any motor split, and enabling lockstep
  (`ENABLE_SIMULATOR_GYROPID_SYNC`, betaflight commit `f11faf414`) flipped no
  failing case to passing. Lockstep is trylock-and-skip (at most one PID
  iteration per FDM packet; exactly one motor packet per FDM packet via
  `updateLock` in every build). Step size shifts the boundary but does not
  save yaw: 0.0025→0.001 made the pitch1510/1522 bracket pass, while
  P23/yaw1504 still fails at 0.001. A suspected SITL motor-scaling authority
  loss was checked and refuted: the packet value is `(PWM-1000)/1000`, full
  0.0–1.0 range. The remaining mechanism is the closed-loop mismatch between
  the Gazebo iris actuator model (P-only rotor velocity loop `vel_p_gain=0.05`,
  hardcoded 838 rad/s command scale in the plugin) and Betaflight's default
  yaw authority — failure is monotonic in yaw P gain and setpoint size, and
  ramping the setpoint (`yaw_rate_limit=120`) keeps the rotor loop linear and
  makes P23 pass.
- The physics step is an acceptance parameter, not a fix (measured
  2026-07-02): axis stability flips with step size in both directions. At
  0.0025 yaw is stable (video yaw4/pitch5/combined3/target-loss all PASS with
  the current binary) but the pitch nudge profile fails (pitch1522 FAIL,
  spread 233). At 0.001 pitch is stable (pitch1522/1530/1540 PASS with
  `--pitch-pid 23,0,0 --yaw-pid 23,0,0 --safe-manual-authority --throttle
  1500`) but the yaw1504 nudge boundary drops to P19 PASS / P20+ FAIL — P22
  is `5/5 FAIL` deterministic at 0.001, so the old "P22 metastability" was an
  artifact of the 0.0025 discretization — and video yaw4 fails (yaw2 passes).
  The takeoff runner default is back to `0.0025`; gates that need 0.001 pin
  it explicitly, and `sitl_video_tracking_check.py` /
  `sitl_synthetic_tracking_check.py` now accept `--max-step-size` to pass it
  through. Always record the step next to any bracket evidence. The durable
  fix is dt-normalizing the plugin rotor velocity PID, not stepping or gain
  hunting (I-only probes at 0.001 fail with the divergence axis moving:
  I1 roll-dominant, I2 pitch-dominant).
- The documented pitch-repro profile was missing flags: it only works with
  `--pitch-pid 23,0,0 --yaw-pid 23,0,0` (default PIDs fail even at 0.001) and
  throttle 1500 (1475 sits on the min-altitude-gain threshold because the
  diagnostics window length varies run to run; altitude gain must always be
  read together with the window length — 75-sample windows span ~40–50 s,
  120-sample video windows ~70–77 s, climb rate is ~0.6 m/s in both).
- Stage 1–4 of `docs/virtual-rc-gazebo-roadmap.md` are measured as of
  2026-07-02, all via virtual RC: stage 1 PASS, stage 2 safe-yaw regression
  `5/5 PASS`, stage 3 brackets measured (see above), stage 4 video gates PASS
  at 0.0025. First measured 5/5 repeatability gate per
  `docs/sitl-flight-readiness-criteria.md` is in place.
- The root cause of the step-dependence is measured (2026-07-02 night): the
  SITL virtual gyro claims 8 kHz, so Betaflight's PID dT and every filter are
  initialized for 125 µs while the lockstep loop actually runs at the FDM
  step (20× off at 0.0025, 8× at 0.001) — filters effectively run at ~1/20 of
  their configured cutoff. An env-gated fix exists:
  `tools/betaflight_looptime_sync.patch` (apply in the betaflight checkout;
  `KENET_SITL_LOOPTIME_US` unset ⇒ bit-for-bit stock) plus the runner flag
  `--sync-betaflight-looptime` (passed through by the video/synthetic
  checkers). Measured with sync: pitch1522@0.0025 flips from FAIL to PASS
  (the pitch killer was the dT/filter skew), but P22/yaw1504@0.001 still
  fails (plant-side), video yaw4@0.0025 flips from PASS to FAIL, and
  transient 945 spread events appear — i.e. the historical 0.0025 stability
  partly rode on the timing bug's accidental over-filtering. Sync is
  EXPERIMENTAL: use only with steps ≥ 2 ms (at 1000 µs the scheduler
  phase-races the FDM unlock and the sim crawls; a tasks.c fast-attempt hack
  was tried and reverted because it starved MSP under Gazebo). Acceptance
  gates therefore stay on the sync-OFF 0.0025 evidence for now; the measured
  path to realistic-timing acceptance is: fix the plugin rotor model
  (dt-normalized/feedforward velocity loop, SDF-parameterized maxRpm), fix
  the sync-at-1ms scheduler starvation properly, then re-bracket everything
  with sync ON (old brackets are not comparable to the sync world).
- The takeoff runner now waits for Betaflight's BOOTGRACE arming-disable flag
  to clear before starting the virtual RC script (`--arming-grace-timeout`,
  default 20 s), and it must do so before the diagnostics poller starts
  because the SITL MSP TCP port accepts one client at a time. Without this, a
  slow boot latches ARM_SWITCH and the run silently never arms. The wait now
  applies to the external RC driver too (mixer-driven video runs hit the same
  latch).
- The sim-specific tune was found 2026-07-03: the sync-world video yaw
  bracket was being killed by the DEFAULT ROLL PID, not the yaw axis. The
  all-axis profile (config import `configs/sync-tune-allaxis-p23.txt` =
  roll P-only 23 + roll/pitch rc_rate 5 / srate 30, plus `--yaw-pid 23,0,0
  --pitch-pid 23,0,0` and safe-yaw rates over MSP) gives, at sync@0.0025:
  **yaw2 5/5 PASS with roll/pitch 0.000 and motor spread 0.000** (first
  fully repeatable command-response gate in any world — this gate's
  acceptance evidence is now sync-world), a metastable saturation band at
  yaw3–yaw5 (~50%: 3/6, 4/6, 3/5; sustained ~777 MSP spread with one motor
  saturated at 2000 during TRACKING even in PASS runs; yaw P19 stays in the
  same regime), and deterministic FAIL at yaw6. Migration decision in
  `docs/sitl-flight-readiness-criteria.md`: gates move to the sync world
  one by one as each measures 5/5 there; the remaining lead for widening
  the band is plant yaw-authority scale, not Betaflight gains.
- Roadmap steps 1-2-3 (plant fix, lockstep robustness, sync-world re-bracket)
  were executed 2026-07-02 night; results in
  `docs/virtual-rc-gazebo-roadmap.md` "Gece 2". Short version: three plant
  knobs now exist (`--iris-velocity-control` + `--iris-motor-time-constant`,
  SDF `maxRotorVelocity`, `--iris-rotor-damping`;
  `tools/gazebo_plugin_velocity_control.patch`) and are measured as NOT
  rescuing the truthful-timing gates — the rotor was already a near-instant
  actuator (τ≈0.3 ms), so the remaining gap is a sim-specific Betaflight tune,
  not plant surgery. A counting-semaphore lockstep wakeup
  (`tools/betaflight_lockstep_sem.patch`) replaced the lossy mutex-unlock
  pattern: sync@0.0025 is now transient-free and the sync-off world is
  unchanged; sync@0.001 still starves the scheduler (open, non-blocking). The
  first sync-world bracket at 0.0025: safe-yaw regression 3/3 PASS, pitch
  1522/1530 (PID23, throttle 1500) PASS — the pitch/yaw step-dependence is
  resolved in the sync world — video neutral and target-loss PASS, video yaw
  bracket yaw2 PASS / yaw3 FAIL (narrower but realistic; widening it is the
  open sim-tune task). Acceptance gates still run sync-OFF until that tune
  exists.
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
