# Kenet SITL — Agent Status & Handoff

Last updated: 2026-07-04. Purpose: let any agent pick up the current, HONEST
state of the drone visual-tracking work without repeating the measurement
mistakes that produced false "PASS" claims earlier. Read this before trusting
any older "works"/"PASS" note in `roadmap-v2.md`, `latest-development.md`, or
`CLAUDE.md`.

Companion docs: `roadmap-v2.md` (goal + stages + measured evidence, with dated
CORRECTIONS at the "A4" sections), `docs/sitl-flight-readiness-criteria.md`
(gate definitions).

---

## TL;DR — what is actually true right now

- **No-command flight is clean.** Arm, prop spin, and throttle→climb work; a
  no-command 40 s hold stays level (roll/pitch 0.0°, motor spread 0).
- **Any sustained yaw command makes the drone spin violently** — measured peak
  **7655 °/s** at 50 Hz. This is independent of yaw authority and of Betaflight
  timing (lockstep on/off, sync on/off all spin); reducing yaw authority just
  moves the instability to roll/pitch.
- **Closed-loop target tracking DOES NOT WORK.** In the correct test the drone
  either spins or hovers in place; it does **not** move toward the target. The
  car drove 32.8 m across the view while the drone moved 0.03 m.
- **Perception works.** The live Gazebo FPV camera feeds the OpenCV tracker
  (30 fps bridge), and the tracker holds a target under smooth motion offline.
- **Do NOT trust any earlier "A4 PASS" / "tracking works" claim.** They came
  from gates that measured the wrong thing (see pitfalls below). They are
  marked INVALID in `roadmap-v2.md`.

---

## CRITICAL: measurement pitfalls that made gates lie (read this first)

These are why earlier runs reported PASS while the drone was spinning / not
tracking. Every new gate must avoid them.

1. **A roll/pitch-only flip gate is blind to a yaw spin.** A drone spinning on
   yaw stays *level* (roll/pitch ≈ 0). Always gate **yaw rate**, not just
   roll/pitch. (`check_yaw_spin` in `sitl_run_quality_check.py`.)
2. **The diagnostics poller samples attitude over MSP at ~2–3 Hz.** A fast spin
   (thousands of °/s) moves thousands of degrees per sample → it **aliases**
   into a flat/slow signal. You cannot see yaw dynamics through the MSP
   diagnostics. Use the 50 Hz Gazebo pose stream (`sitl_yaw_monitor.py`).
   Treat any yaw-rate number from diagnostics as an aliased **lower bound**.
3. **"Moved in a direction" / heading is NOT a tracking test.** A spinning or
   hovering drone yaws through every heading and proves nothing. The real
   track/control test is that the **drone POSITION converges on the target**
   (distance closes for a static target; stays bounded while the drone
   translates *with* a moving target). Use `sitl_track_truth.py`.
4. **These signals do NOT prove tracking:** `target_found` ratio high, mixer
   `max_abs_yaw_delta > 0`, "tracked box swept the frame 0→640", roll/pitch
   level. A spinning camera makes the world sweep past → identical to
   "following" in those metrics. Only position convergence counts.
5. **Headless ≠ GUI, and camera+sync runs have unstable RTF** (measured 0.46–
   1.10). The GUI render load changes timing and can trigger the spin sooner.
   A headless PASS does not guarantee GUI behavior.
6. **MSP diagnostics polling drops samples under camera/mixer load** (~25–50%).
   That is a monitoring-contention artifact, not a flight fault — Gazebo pose
   is still 50 Hz and authoritative. Relax MSP-continuity for camera runs
   (`--msp-min-connected-fraction`), but never relax the yaw/position gates.

---

## Status of the 9 core metrics (honest)

| # | Metric | Status |
|---|--------|--------|
| 1 | Arm/disarm | Works (arm verified; disarm gate is report-only until the diagnostics window covers the disarm phase). |
| 2 | Props spin on arm | Works. |
| 3 | Throttle → climb | Works (no-command). |
| 4 | Correct R/P/Y response | **Broken.** Sustained yaw → spin (7655 °/s). Direction is correct, magnitude/coupling is not. |
| 5 | Commands reach FC (MSP) | Verified. Override write-path proven correct by loopback (`sitl_msp_override_loopback.py`). |
| 6 | Tracking PID exists & works | PID exists; convergence **not achieved** — see metric 4/closed-loop. Offline settling tests pass against a toy plant only. |
| 7 | Tracker robust under motion | Works offline (`tracker_motion_benchmark.py`) and on live camera (found, 0 errors). |
| 8 | Targets move in sim | Works — `sitl_target_mover.py` (set_pose), measured displacement. |
| 9 | Sim camera → tracker | Works — `kenet/gz_camera.py` bridge, `sitl_gz_camera_probe.py` (30 fps). |

**Closed-loop track-and-approach (the project goal): DOES NOT WORK YET.**

---

## The gates to use (and trust)

Trust these because they measure the right thing and were validated to catch
the real failures.

- Offline suite: `fpv_env/bin/python -m pytest -q` (384 tests).
- **Per-run flight quality (from logs):** `tools/sitl_run_quality_check.py`
  — validity (armed, clean first sample, MSP), attitude flip, **yaw spin**,
  RTF/cadence health, window-normalized climb, disarm, signed yaw. Correctly
  classifies clean=PASS, flip/spin=FAIL, never-armed=INVALID.
- **Real-time yaw / spin (50 Hz):** `tools/sitl_yaw_monitor.py` — subscribes to
  Gazebo pose, prints live yaw + yaw rate, SPIN verdict. This is how you SEE a
  spin.
- **Track/control acceptance (THE real test):** `tools/sitl_track_truth.py` —
  drone POSITION converges on target (distance closes / stays bounded + drone
  translates + no spin). A hovering/spinning drone FAILS this.
- Perception: `tools/sitl_gz_camera_probe.py` (camera bridge live),
  `tools/tracker_motion_benchmark.py` (tracker under motion, offline).

Do NOT reintroduce heading/`max_abs_yaw_delta`-only or roll/pitch-only gates as
proof of tracking.

---

## Tools built for this (all have offline tests)

- `kenet/gz_camera.py` — Gazebo camera topic → OpenCV bridge. Camera source
  `gz:/kenet/fpv_camera` via `kenet.camera.create_camera_capture`.
- `tools/sitl_run_quality_check.py` — offline per-run/ chain verdicts.
- `tools/sitl_yaw_monitor.py` — 50 Hz real-time yaw-rate / spin monitor.
- `tools/sitl_track_truth.py` — 50 Hz position-convergence track/control gate.
- `tools/sitl_target_mover.py` — set_pose target driver + ground-truth log.
- `tools/sitl_moving_target_track_check.py` — orchestrator: runs the
  camera-tracking checker + target mover together. (Its own verdict is being
  migrated to position convergence; its yaw-spin gate is in place.)
- `tools/sitl_msp_override_loopback.py` — MSP override write-path gate.
- `tools/sitl_gz_camera_probe.py`, `tools/tracker_motion_benchmark.py`,
  `tools/sitl_plot_tracking.py` — perception + visualization.

---

## Known root issues to fix next (verify against position, not heading)

1. **Yaw spin — ROOT CAUSE FOUND (2026-07-04).** The spin is a **loop-gain
   instability in Betaflight's yaw rate-PID**, not the command magnitude. With
   the profile's `yaw P = 23` (too hot for this plant's yaw authority), any yaw
   excitation — even a ±2 µs command — diverges to a runaway spin (measured
   50 Hz peaks 1125–7655 deg/s). Lowering the Betaflight yaw rate-PID P fixes
   it: **P=6 → peak 2.5 deg/s, P=3 → peak 54 deg/s, both stable, roll/pitch
   ~0**. Note it is NOT the Kenet mixer `--yaw-limit` (command clamp): ±2 still
   spun at P=23. Lever is the FC rate-loop P (set via the checker/launcher
   `--yaw-pid`, or `sitl_moving_target_track_check.py --betaflight-yaw-pid`,
   now defaulting to `6,0,0`).
   **Remaining trade-off (open):** at the stable low P the yaw is too weak to
   actually turn/track — the drone holds heading (turned only ~2° while the
   commanded stick saturated). So: high P tracks-but-spins, low P
   stable-but-can't-track. Next tuning: raise yaw authority at a stable loop
   gain (bigger yaw-rate setpoint / larger `--yaw-limit`, or add an I term to
   close steady-state error) without re-triggering divergence, then verify with
   `sitl_track_truth.py` (position convergence). The earlier "cut yaw authority
   → flip moves to roll" result was at the hot P=23 regime; re-evaluate plant
   knobs only at the stable gain.
2. **Approach axis produces no translation.** With `--forward-limit` enabled the
   mixer commands a small forward pitch but the drone does not move toward the
   target. The forward/approach control (bbox-width → pitch) must produce actual
   position change.
3. **Any fix MUST be verified with `sitl_track_truth.py`** (drone position
   converges on the target), never with heading/`target_found`/roll-pitch alone.

---

## Environment & run hygiene (important)

- Python: `fpv_env/bin/python`. Do not install into the system `python3`.
- gz bindings live in system `/usr/lib/python3/dist-packages` (Python 3.12).
  Use `/usr/bin/python3` for gz tools, or `fpv_env` with that path appended
  (the tools do this automatically).
- Betaflight binary (`~/betaflight/obj/main/betaflight_SITL.elf`) is the
  Jul-3 build with lockstep (`ENABLE_SIMULATOR_GYROPID_SYNC=1`) + the semaphore
  and env-gated looptime patches. **The documented "safe-yaw 5/5" evidence was
  measured on a DIFFERENT (pre-lockstep) binary and does not reproduce on this
  one** — do not cite old brackets against the current binary.
- **Port hygiene before every live run:** ensure 5761 and 9002–9004 are free
  and no stale `betaflight_SITL` / `gz sim` processes remain — a lingering
  Betaflight on 5761 silently blocks startup and yields empty logs. Clean with:
  ```bash
  pkill -9 -f "gz sim|betaflight_SITL|run_gazebo_betaflight|moving_target|track_truth"
  ss -tlnp | grep -E "5761|900[234]"   # must be empty
  ```
- `launch-fpv-sim.sh` is **manual keyboard flying only** — no tracker, no
  target mover. It cannot demonstrate autonomous tracking.
  Updated 2026-07-04 for safe manual control: it now applies the stable yaw
  rate-PID (`--yaw-p 6`, was 23) and clamps manual yaw authority
  (`--yaw-authority 10` us, both keyboard and RC) so a held a/d key cannot spin
  the drone. Measured: sustained +10 us yaw at P=6 → peak 6.8 deg/s, no spin;
  +20 us still spins. So yaw is intentionally weak (safe, barely turns);
  roll/pitch/throttle are full. Real yaw authority still needs the plant fix —
  raising `--yaw-authority` will spin it. All limits are CLI-configurable
  (`--yaw-p --pitch-p --yaw-rate --rate-limit --yaw-authority --stick-step`).

---

## Reproduce the key findings

```bash
cd /home/gz/fpv-test

# 1. Offline suite
fpv_env/bin/python -m pytest -q

# 2. See the spin at 50 Hz: start a tracking run, then attach the monitor
fpv_env/bin/python tools/sitl_moving_target_track_check.py \
    --model car_front_1 --speed 0.5 --z 1.5 --cross-amplitude 6 --json &
sleep 26
/usr/bin/python3 tools/sitl_yaw_monitor.py --world betaloop_demo --model iris \
    --duration 75 --max-yaw-rate 90        # expect: SPIN DETECTED, peak thousands of deg/s

# 3. The correct track test: drone position vs target (expect FAIL today)
#    (run alongside a tracking flight; measures distance closing + translation + spin)
/usr/bin/python3 tools/sitl_track_truth.py --world betaloop_demo \
    --drone iris --target car_front_1 --duration 70 --min-approach-m 2
```

Evidence logs from 2026-07-04 live runs are under `logs/sitl/`:
`*-a4-track-*` (tracking), `*-yaw-monitor-*` (50 Hz yaw, shows 7655 °/s),
`*-track-truth-*` (position convergence, shows no approach).

---

## One-line summary for the next agent

Perception and no-command flight work; the closed-loop track-and-approach does
not — the drone spins or hovers instead of moving toward the target. The honest
acceptance test is `sitl_track_truth.py` (position convergence). Fixing the
sustained-yaw spin and the missing approach translation is the open work, and
must be judged by position, not heading.
