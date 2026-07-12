# Game Screen Sandbox Phase Checklist

This is the local execution checklist for `simple_game_sitl.md`.

The sandbox is intentionally isolated from the Gazebo/Betaflight launch path.
Keep changes under `experiments/game_screen_sandbox/` and runtime evidence under
`logs/game_screen_sandbox/`.

Current verified state on 2026-07-05:

- Synthetic S0-S4 phase gates pass.
- Real X11 crop capture passes with `ffmpeg/x11grab`.
- X11 window-title to capture-region resolution passes with `xwininfo`.
- Real `uinput` smoke passes after installing
  `experiments/game_screen_sandbox/requirements-input.txt` into `fpv_env`.
- No Gazebo/Betaflight launch path is required for these gates.

Current automated gate runner:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/phase_runner.py \
  --duration 2 \
  --hz 10 \
  --run-id sandbox-phase
```

This runner proves the synthetic S0-S4 core only. Real game/window capture and
real input remain separate manual gates until a simulator is selected.

One-command independent acceptance runner:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/sandbox_acceptance_runner.py \
  --run-id sandbox-acceptance \
  --include-simple-window
```

Without `--include-simple-window`, it still writes the full S0-S14 synthetic
phase report and a decision report, but the decision remains `WAITING` because
real-window smoke evidence is intentionally missing.

Isolation audit only:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/isolation_audit.py --json
```

This checks sandbox Python imports and Gazebo/Betaflight process boundaries
without launching the main Gazebo/Betaflight path.

Repo-local simple target window:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/simple_target_game.py \
  --duration 2 \
  --fps 10 \
  --static-target \
  --save-frame logs/game_screen_sandbox/simple-game-frame.png \
  --report-path logs/game_screen_sandbox/simple-game-report.json
```

This OpenCV window is not a flight simulator. It is a tiny target screen source
used to validate capture/tracker/PID gates before choosing a real game or FPV
simulator. Start with `--static-target` for real-window capture smoke; use S8
for deterministic moving-target regression.

One-command local real-window smoke:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/simple_window_smoke.py \
  --run-id sandbox-simple-window
```

This opens the target window, resolves the X11 content region, captures it, and
runs the S7 combined dry-run gate.

When the OpenCV window is visible, prefer the 640x480 content region before
feeding it to `phase_runner.py`:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/x11_window.py \
  --title "Kenet Simple Target Game" \
  --preferred-size 640x480
```

After a game/sim window is visible, include a real X11 crop with:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/phase_runner.py \
  --capture-region 0,0,640,480 \
  --capture-backend ffmpeg \
  --duration 2 \
  --hz 10 \
  --run-id sandbox-real-capture
```

Or resolve the crop from a visible X11 window title:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/x11_window.py --list

fpv_env/bin/python experiments/game_screen_sandbox/bbox_tool.py \
  --window-title pr0p \
  --capture-backend ffmpeg \
  --frame-path logs/game_screen_sandbox/pr0p-target-frame.png

fpv_env/bin/python experiments/game_screen_sandbox/phase_runner.py \
  --capture-window-title pr0p \
  --capture-backend ffmpeg \
  --tracker-bbox-file logs/game_screen_sandbox/pr0p-target-bbox.json \
  --include-real-loop \
  --duration 2 \
  --hz 10 \
  --run-id sandbox-pr0p-capture
```

External game/sim window preflight after bbox selection:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/sandbox_status_report.py \
  --window-title pr0p \
  --bbox-file logs/game_screen_sandbox/pr0p-target-bbox.json
```

This is the first low-cost resume command. It only reads the latest reports,
target bbox file, X11 window state, uinput probe, and forbidden-process state;
it does not launch capture, simulator, Gazebo, Betaflight, or OS input.
If signed axis direction is required, add `--axis-expected-shifts` and
`--axis-min-shift-px`; the generated resume commands preserve those flags.

If the requested title does not match a visible simulator window, run
`external_operator_preflight.py` with `--max-window-candidates`. Each candidate
includes a title-based rerun command, a region-based bbox capture command, and a
region-based `external_dry_run_sequence.py` command for the case where the
window title is unreliable. The same report also includes
`candidate_action_plan`: with no bbox it recommends region frame capture; with a
current bbox it recommends the region dry-run sequence. These candidate actions
never send `uinput` and must be manually confirmed against the visible simulator
view before execution. Tooling/editor/shell windows such as VS Code or
`user@host: ~` terminals are listed as `excluded_candidates` and do not become
recommended candidate actions, even when their title contains the simulator
text.

For a guarded handoff, use `external_candidate_action_runner.py`. Without
`--candidate-index`, it only writes the candidate plan and returns `WAITING`.
With one selector plus `--ack-candidate --execute-safe`, it executes exactly one
selected safe non-`uinput` candidate action. Prefer `--candidate-window-id` for
stable selection; `--candidate-index`, exact `--candidate-title`, and unique
`--candidate-title-contains` are also supported. Ambiguous title-substring
matches are rejected. Obvious tooling/editor windows are rejected too, even when
their title contains the simulator text. This is the intended bridge from
operator preflight to either region bbox capture or region dry-run sequence.
After the operator inspects the captured frame, the same runner accepts
`--candidate-bbox X,Y,W,H`; this converts the selected candidate action into a
region-based bbox write command and writes the bbox file without returning to a
title-based capture path. The bbox is interpreted inside the selected candidate
region; out-of-region bbox values are rejected before any capture/write command
runs.
If a candidate action runs, the runner immediately writes a post-action operator
preflight report, so the handoff includes before/after readiness evidence.

When window and bbox are ready, the dry-run sequence runs status, preflight, and
follow dry-run in order. It still does not use `uinput`:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/external_dry_run_sequence.py \
  --capture-window-title pr0p \
  --capture-backend ffmpeg \
  --tracker-bbox-file logs/game_screen_sandbox/pr0p-target-bbox.json \
  --duration 2 \
  --hz 10 \
  --enable-pitch \
  --desired-target-width 120 \
  --run-id pr0p-dry-sequence
```

Its report now records `sequence_steps` plus `live_readiness`. `live_readiness`
becomes `READY_FOR_OPTIONAL_LIVE_INPUT` only after status, preflight, and follow
dry-run all pass.
With `--capture-region`, the setup gate intentionally skips `target_window` and
requires only the bbox file plus Gazebo/Betaflight process-boundary audit before
preflight/follow dry-run can start.

Live virtual-RC readiness is a separate explicit gate. It first reruns the dry
sequence and then performs the S5-real axis-response sweep only when
`--ack-live-input` is provided:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/external_live_input_readiness.py \
  --capture-window-title pr0p \
  --capture-backend ffmpeg \
  --tracker-bbox-file logs/game_screen_sandbox/pr0p-target-bbox.json \
  --duration 2 \
  --hz 10 \
  --enable-pitch \
  --desired-target-width 120 \
  --ack-live-input \
  --axis-sweep-axes yaw,pitch \
  --run-id pr0p-live-input-readiness
```

This writes `EXTERNAL_LIVE_INPUT_READY` only when the dry sequence and visual
axis response both pass. Without the acknowledgement, the report remains
`WAITING` and no virtual RC command is sent.

Status promotion requires embedded live-input proof, not only the top-level
`EXTERNAL_LIVE_INPUT_READY` value. The matching report must include S5-real
`UInputAdapter` metrics with `real_input: true` and `axis_sweep: true`; if
signed axis expectations are requested, the report must carry the same
expectation and minimum shift.

When axis inversion is a risk, require signed visual motion before promoting
live input. Example:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/external_live_input_readiness.py \
  --capture-window-title pr0p \
  --capture-backend ffmpeg \
  --tracker-bbox-file logs/game_screen_sandbox/pr0p-target-bbox.json \
  --duration 2 \
  --hz 10 \
  --enable-pitch \
  --desired-target-width 120 \
  --ack-live-input \
  --axis-sweep-axes yaw,pitch \
  --axis-expected-shifts yaw:+x,pitch:-y \
  --axis-min-shift-px 1 \
  --run-id pr0p-live-input-readiness-signed
```

This still uses the same ack-gated S5-real axis sweep. It adds
`response_shift_x_px`, `response_shift_y_px`, and `direction_status` to the
axis report, so an inverted axis fails before bounded live follow starts.

Bounded live follow is gated by a separate sequence that rechecks readiness
before sending follow-loop virtual RC:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/external_live_follow_sequence.py \
  --capture-window-title pr0p \
  --capture-backend ffmpeg \
  --tracker-bbox-file logs/game_screen_sandbox/pr0p-target-bbox.json \
  --duration 1 \
  --hz 10 \
  --enable-pitch \
  --desired-target-width 120 \
  --ack-live-input \
  --live-max-duration 2 \
  --run-id pr0p-live-follow-sequence
```

This writes `EXTERNAL_LIVE_FOLLOW_COMPLETE` only when both live-input readiness
and the bounded live follow pass.

The status-only resume command can require that live-input evidence too:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/sandbox_status_report.py \
  --window-title pr0p \
  --bbox-file logs/game_screen_sandbox/pr0p-target-bbox.json \
  --require-live-input \
  --axis-expected-shifts yaw:+x,pitch:-y \
  --axis-min-shift-px 1
```

Status evidence is tied to the current bbox file. If the user reselects the
target, older preflight/follow/readiness reports for the same window title no
longer satisfy `READY`.

The status report also writes `resume_commands`, a machine-readable list of the
next exact commands. Each entry includes `safety_class`, `expected_status`, and
`unblocks`; commands that can send virtual RC are marked `sends_uinput: true`.
Entries can include `requires_pass`; the runner blocks the command until every
listed status item is `PASS`.
If the target bbox is missing, the list includes both a manual coordinate
template and `interactive_bbox_select`. The interactive command is marked
`requires_user_interaction: true`, so the runner will not open its ROI UI
automatically.

For resumable automation, use the plan-only runner first:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/sandbox_resume_runner.py \
  --window-title pr0p \
  --bbox-file logs/game_screen_sandbox/pr0p-target-bbox.json \
  --require-live-follow
```

Add `--execute-safe` only when the listed non-uinput/non-edit commands should
run automatically. Commands marked `sends_uinput: true` remain blocked unless
both `--execute-live-input` and `--ack-live-input` are present. If any command
runs, the runner writes a post-status report and uses that evidence to promote
the result to ready when the requested gate is satisfied.
Manual UI commands also remain blocked in the runner. Run
`bbox_tool.py --interactive-select` directly only when the simulator window is
visible and you are ready to choose the target.
Dependency-gated commands also remain blocked until their `requires_pass`
status items are ready. This prevents live-input or live-follow commands from
running before the dry capture/tracker/PID gates are proven.
The runner's `resume_decision` field summarizes the first operator action,
first unmet dependency, and first planned command for handoff.
The runner's `resume_step_summary` field counts step states, live-input-capable
commands, executed commands, and the first planned/blocked/unmet/failing step.
The status report's `readiness_ladder` field gives the same state as S0-S5
stages, so a handoff can identify whether the next block is target setup,
dry-run evidence, binding, live-input readiness, or bounded live follow.
The `evidence_freshness` field records artifact age and marks evidence as
`FRESH`, `STALE`, or `MISSING`, so old reports do not look silently current.
Use `--require-fresh-evidence --evidence-stale-after-s 3600` when stale or
missing required evidence should block `READY`; without that flag, freshness is
reported as advisory handoff context.

Before a real pr0p/SITL Forge attempt, use the operator preflight for a single
status-only handoff view:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/external_operator_preflight.py \
  --window-title pr0p \
  --bbox-file logs/game_screen_sandbox/pr0p-target-bbox.json \
  --require-live-follow \
  --require-fresh-evidence \
  --evidence-stale-after-s 3600 \
  --axis-expected-shifts yaw:+x,pitch:-y \
  --axis-min-shift-px 1
```

It combines the current status report with operator command states and writes
`operator_preflight_decision`, so the next human action and recommended command
are visible without starting capture or input. It also writes
`window_discovery`, which lists visible X11 application-window candidates when
the requested `--window-title` does not match. Each candidate includes exact
rerun, title-based bbox-frame, and region-based bbox-frame commands so the next
attempt can use the selected title or screen region without hand-building shell
arguments.

When dry evidence is ready but live-input response is not, the resume list also
includes `rc_binding_assistant.py` for the simulator `Controls -> RC Channels`
page. It is a live virtual-RC command and is therefore blocked unless both
live-input acknowledgement flags are present. Dry-run binding reports are not
accepted as RC channel binding evidence; status matching requires a live
`uinput`, acknowledged, neutralized binding report for the current window and
bbox.

After `EXTERNAL_LIVE_FOLLOW_COMPLETE`, use the stronger status gate:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/external_target_acceptance.py \
  --window-title pr0p \
  --bbox-file logs/game_screen_sandbox/pr0p-target-bbox.json \
  --mode live-follow \
  --axis-expected-shifts yaw:+x,pitch:-y \
  --axis-min-shift-px 1 \
  --evidence-stale-after-s 3600
```

This is a status-only external acceptance gate. It returns
`EXTERNAL_TARGET_READY` only when the nested status report is `READY`; otherwise
it remains `WAITING` or `REJECT` and reports the first S0-S5 block. It also
records a before/after Gazebo/Betaflight process-boundary audit and an
`acceptance_decision` summary for handoff. The same report also exposes an
`operator_command_queue`, so the next command, its safety class, and any
`requires_pass` gate are visible without digging into nested status JSON. Each
queue entry also includes `command_state` and `unmet_requires_pass`, so blocked
commands explain themselves.
`operator_command_summary` aggregates those states and points to the first
available, blocked, and ack-required command.

```bash
fpv_env/bin/python experiments/game_screen_sandbox/sandbox_status_report.py \
  --window-title pr0p \
  --bbox-file logs/game_screen_sandbox/pr0p-target-bbox.json \
  --require-live-follow
```

```bash
fpv_env/bin/python experiments/game_screen_sandbox/external_window_preflight.py \
  --capture-window-title pr0p \
  --capture-backend ffmpeg \
  --tracker-bbox-file logs/game_screen_sandbox/pr0p-target-bbox.json \
  --run-id pr0p-window-preflight
```

This writes `EXTERNAL_WINDOW_READY_DRY`, `WAITING`, or `REJECT` and covers
S1-real capture, S2-real tracker, and S7-real combined yaw+pitch dry-run.

After this dry preflight passes, the same entrypoint can also verify that the
selected simulator window actually reacts to virtual RC channels:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/external_window_preflight.py \
  --capture-window-title pr0p \
  --capture-backend ffmpeg \
  --tracker-bbox-file logs/game_screen_sandbox/pr0p-target-bbox.json \
  --include-axis-sweep \
  --ack-live-input \
  --axis-sweep-axes yaw,pitch \
  --run-id pr0p-window-preflight-axis
```

This promotes the verdict to `EXTERNAL_WINDOW_READY_LIVE_INPUT` only when S5-real
also passes. If S1/S2/S7 pass but S5-real waits, first check the simulator/game
`Controls -> RC Channels` or axis binding page; that usually means the virtual
RC device exists but the game is not listening to those channels.

Real virtual input is a separate opt-in gate:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/phase_runner.py \
  --include-real-input \
  --uinput-smoke \
  --input-yaw 0.05 \
  --input-hold-seconds 0 \
  --run-id sandbox-uinput-smoke
```

Live yaw-only input is an explicit opt-in gate after S4-real dry-run passes:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/phase_runner.py \
  --capture-window-title pr0p \
  --capture-backend ffmpeg \
  --tracker-bbox-file logs/game_screen_sandbox/pr0p-target-bbox.json \
  --include-live-loop \
  --ack-live-input \
  --duration 1 \
  --hz 5 \
  --live-max-duration 2 \
  --run-id sandbox-pr0p-loop-live
```

For an external game/sim window, use the session runner once bbox selection and
preflight are green. Dry-run does not send OS joystick commands:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/external_follow_session.py \
  --capture-window-title pr0p \
  --capture-backend ffmpeg \
  --tracker-bbox-file logs/game_screen_sandbox/pr0p-target-bbox.json \
  --duration 2 \
  --hz 10 \
  --enable-pitch \
  --desired-target-width 120 \
  --run-id pr0p-follow-dry
```

The same runner can send real virtual RC only with explicit acknowledgement and
a duration cap:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/external_follow_session.py \
  --capture-window-title pr0p \
  --capture-backend ffmpeg \
  --tracker-bbox-file logs/game_screen_sandbox/pr0p-target-bbox.json \
  --duration 1 \
  --hz 10 \
  --uinput \
  --ack-live-input \
  --live-max-duration 2 \
  --run-id pr0p-follow-live
```

Latency measurement is S5-real and can be run through the same runner:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/phase_runner.py \
  --capture-window-title pr0p \
  --capture-backend ffmpeg \
  --include-latency-probe \
  --latency-uinput \
  --ack-live-input \
  --input-yaw 0.05 \
  --duration 1 \
  --hz 10 \
  --run-id sandbox-pr0p-latency
```

Axis mapping sweep, after the sim/game window is open and virtual RC is
allowed:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/phase_runner.py \
  --capture-window-title pr0p \
  --capture-backend ffmpeg \
  --include-latency-probe \
  --latency-axis-sweep \
  --latency-axes yaw,pitch,roll,throttle \
  --latency-uinput \
  --ack-live-input \
  --duration 1 \
  --hz 10 \
  --run-id sandbox-pr0p-axis-sweep
```

## S0 - Environment And Game Choice

Goal: choose the first offline/local simulator and verify screen/input basics.

Automated status:

- `phase_runner.py` records desktop/capture/input prerequisites.
- It marks the synthetic core usable even when no real game has been selected.
- `isolation_audit.py` verifies that sandbox Python imports do not depend on
  Gazebo/Betaflight modules and reports forbidden process presence/delta.

Gate:

- Desktop session can show the game window.
- Capture backend is selected: synthetic, video, `mss`, ffmpeg/x11grab, or
  v4l2loopback.
- Input backend is selected: dry-run, physical controller, keyboard, or
  `uinput`.
- Neutral/stop path is known before real input is enabled.

## S1 - Capture Smoke

Goal: prove frames can be read without involving Gazebo or Betaflight.

Commands:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/capture_window.py \
  --synthetic \
  --duration 2 \
  --fps 10
```

Gate:

- Nonblank frames.
- Stable frame size.
- Reported FPS is acceptable for tracker work.

Automated status:

- `phase_runner.py` runs this with `SyntheticFrameSource`.
- `phase_runner.py --capture-region ... --capture-backend ffmpeg` runs the same
  gate on a real X11 screen crop without needing `mss`.
- `bbox_tool.py --window-title ...` captures a target-selection frame and can
  draw/validate the chosen bbox.
- `bbox_tool.py --interactive-select` captures one frame and lets the user pick
  the target bbox in an OpenCV ROI UI; cancel returns `WAITING`.
- `external_window_preflight.py` combines bbox-selection waiting state with
  S1-real/S2-real/S7-real readiness for a selected external game window.

## S2 - Tracker Smoke

Goal: prove the tracker can follow a target in captured frames while sending no
input.

Automated status:

- `phase_runner.py` initializes KCF on the known synthetic target bbox and
  checks `found_ratio`.
- `phase_runner.py --tracker-bbox x,y,w,h` adds S2-real and checks that real
  captured frames can initialize/update the tracker.
- `bbox_tool.py --bbox x,y,w,h` or `bbox_tool.py --interactive-select` is the
  repeatable pre-check before S2-real.
- `phase_runner.py --tracker-bbox-file ...` can consume the bbox JSON produced
  by `bbox_tool.py`.

Gate:

- `found_ratio >= 0.90` on a simple sequence.
- Bbox center and size are logged.
- Target-loss events are visible in the log.

## S3 - Virtual Input Smoke

Goal: prove commands can be sent and safely neutralized.

Automated status:

- `phase_runner.py` sends a dry-run yaw/pitch command and verifies neutral on
  close.
- `phase_runner.py --include-real-input` reports S3-real uinput prerequisites.
- `phase_runner.py --include-real-input --uinput-smoke` creates a real virtual
  controller only when explicitly requested.
- `rc_binding_assistant.py --window-title <title> --tracker-bbox-file <bbox>
  --uinput --ack-live-input --axes <axis>` pulses one virtual joystick axis
  while the simulator/game `Controls -> RC Channels` binding row is selected,
  and writes window/bbox context for status matching.

Gate:

- Dry-run command mapping passes in tests.
- Real input is enabled only after manual neutral/kill-switch check.
- Each required simulator RC channel can be bound one axis at a time before
  running the visual S5-real axis-response gate.
- Small yaw/pitch commands move the game in the expected direction.

## S4 - Yaw-Only Closed Loop

Goal: first minimal visual-servo loop.

Command:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/screen_tracking_loop.py \
  --synthetic \
  --duration 2 \
  --hz 10 \
  --dry-run
```

Gate:

- Loop runs without Gazebo/Betaflight.
- Commands remain bounded.
- Target stays found in the simple sequence.
- Horizontal error and command magnitude are reported.

Automated status:

- `phase_runner.py` runs capture -> tracker -> PID -> dry-run input and writes
  JSONL evidence.
- `phase_runner.py --include-real-loop --tracker-bbox x,y,w,h` runs the same
  yaw-only path on real captured frames with `DryRunInputAdapter`. It does not
  send joystick commands to the game.
- `phase_runner.py --include-live-loop --ack-live-input` is the first real
  uinput yaw-only gate. It is duration-capped and remains separate from S4-real.

## S4b - Yaw Hardening, Only If Needed

Do this only if S4 shows runaway, saturation, or unstable turning.

Add:

- Runaway guard.
- Kill-switch integration.
- Lower yaw output limits.
- Larger deadband or stronger rate limit.
- End-to-end yaw latency measurement.

## S5 - Latency Measurement

Goal: measure capture -> tracker -> controller -> input -> rendered-motion
delay after S4 is stable enough to evaluate.

Automated status:

- `latency_probe.py` measures command timestamp to first visual frame response.
- `phase_runner.py --include-latency-probe` adds S5-real.
- `--latency-axis-sweep` probes selected axes one by one and reports
  `axis_statuses`, `passed_axes`, and `waiting_axes`.
- `--latency-axis-expected-shifts yaw:+x,pitch:-y` optionally upgrades S5-real
  from "image changed" to "image moved in the expected signed direction".
- Real input measurement requires `--latency-uinput --ack-live-input`.
- If S3-real passes but S5-real does not show a visual response, check the
  simulator/game `Controls -> RC Channels` or axis binding page before tuning
  PID or tracker logic.

Gate:

- Command timestamps and frame timestamps are in the same log.
- First visible image response after a yaw step can be measured.

## S6 - Pitch / Approach First Pass

Goal: try bbox width as the first simple approach signal.

Command:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/phase_runner.py \
  --capture-window-title pr0p \
  --capture-backend ffmpeg \
  --tracker-bbox-file logs/game_screen_sandbox/pr0p-target-bbox.json \
  --include-real-approach \
  --desired-target-width 120 \
  --duration 2 \
  --hz 10 \
  --run-id sandbox-pr0p-approach-dry
```

Automated status:

- `phase_runner.py --include-real-approach` adds S6-real.
- S6-real uses `DryRunInputAdapter`; it does not send joystick commands.
- The report includes desired bbox width, observed bbox width, width error,
  pitch command bounds, found ratio, and neutralization.

Dry-run gate:

- Tracker remains found.
- Width error is reported against `desired_target_width`.
- Pitch command remains bounded.
- Input neutralizes at the end.

Live/game-response gate, after dry-run:

- Bbox width moves toward the desired value.
- Yaw centering does not degrade badly.
- Pitch command remains bounded.

## S6b - Approach Metric Hardening, Only If Needed

Do this only if S6 proves bbox width is noisy or misleading.

Add one or more secondary signals:

- Smoothed bbox scale trend.
- Optical flow.
- Simulator telemetry if available.
- Too-close neutral/retreat rule.

## S7 - Combined Yaw + Pitch

Goal: keep target centered while approaching.

Command:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/phase_runner.py \
  --capture-window-title pr0p \
  --capture-backend ffmpeg \
  --tracker-bbox-file logs/game_screen_sandbox/pr0p-target-bbox.json \
  --include-real-combined \
  --desired-target-width 120 \
  --duration 2 \
  --hz 10 \
  --run-id sandbox-pr0p-combined-dry
```

Automated status:

- `phase_runner.py --include-real-combined` adds S7-real.
- S7-real uses `DryRunInputAdapter`; it does not send joystick commands.
- The report includes horizontal error, bbox width error, yaw/pitch command
  bounds, found ratio, and neutralization.

Dry-run gate:

- Tracker remains found.
- Yaw command remains bounded.
- Pitch command remains bounded.
- Input neutralizes at the end.

Live/game-response gate, after dry-run:

- 60 second run.
- `found_ratio >= 0.85`.
- Horizontal error, approach trend, saturation, and target-loss events are
  reported.
- Target loss returns input to neutral.

## S8 - Moving Target

Goal: follow a moving target in the chosen game/sim.

Command:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/phase_runner.py \
  --include-moving-target \
  --duration 2 \
  --hz 10 \
  --min-moving-target-motion 30 \
  --run-id sandbox-moving-target
```

Automated status:

- `phase_runner.py --include-moving-target` adds S8.
- S8 uses a deterministic synthetic moving target and `DryRunInputAdapter`.
- The report includes center motion, found ratio, loss events, yaw/pitch
  command bounds, and neutralization.

Pre-game gate:

- Target center motion is above `--min-moving-target-motion`.
- Tracker remains found.
- Loss events are reported.
- Yaw/pitch commands remain bounded.
- Input neutralizes at the end.

Game-specific gate, after a simulator target exists:

- At least 30 seconds of moving-target evidence.
- Loss count, center error, and command bounds are reported.

## S9-game - In-Process Game Camera Control

Goal: prove PID output is not only logged, but applied to a simple game camera
and reduces target centering error.

Command:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/phase_runner.py \
  --include-moving-target \
  --include-game-dynamics \
  --include-range-dynamics \
  --duration 4 \
  --hz 20 \
  --run-id sandbox-game-dynamics
```

Automated status:

- `phase_runner.py --include-game-dynamics` adds `S9-game`.
- `S9-game` renders the repo-local simple target game in-process.
- The tracker initializes from the first rendered target bbox.
- PID yaw output is applied back to `step_state()` as camera yaw.
- The report includes initial error, final error, reduction ratio, found ratio,
  loss events, command bounds, and a JSONL sample log.

Gate:

- Target starts with a visible horizontal offset.
- Tracker remains found.
- Final horizontal error is below the configured bound.
- Error reduction ratio exceeds the configured bound.
- Yaw command remains bounded.
- No Gazebo, pr0p, Betaflight, screen capture, or OS input is used.

## S10-range - In-Process Approach Control

Goal: prove forward/pitch PID output is not only logged, but changes apparent
target size in the simple sim and reduces bbox-width error.

Command:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/phase_runner.py \
  --include-range-dynamics \
  --duration 5 \
  --hz 20 \
  --run-id sandbox-range-dynamics
```

Automated status:

- `phase_runner.py --include-range-dynamics` adds `S10-range`.
- `S10-range` renders the repo-local simple target game in-process with a
  synthetic apparent-width/range state.
- A simple color bbox detector measures the rendered target width from pixels.
- PID yaw is applied to the camera and PID forward/pitch is applied to target
  apparent width.
- The report includes center error, width error, reduction ratios, found ratio,
  command bounds, and a JSONL sample log.

Gate:

- Target starts with visible horizontal and width errors.
- Tracker/detector remains found.
- Final horizontal error is below the configured bound.
- Final width error is below the configured bound.
- Center and width error reductions exceed configured bounds.
- Yaw/pitch commands remain bounded.
- No Gazebo, pr0p, Betaflight, screen capture, or OS input is used.

## S11-handoff - Manual To Autonomous Handoff

Goal: prove the first user flow: manual centering happens before the follow
command, and tracker/PID control starts only after follow is requested.

Command:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/phase_runner.py \
  --include-handoff-dynamics \
  --duration 6 \
  --hz 20 \
  --run-id sandbox-handoff-dynamics
```

Automated status:

- `phase_runner.py --include-handoff-dynamics` adds `S11-handoff`.
- The sim starts with the target visibly off-center.
- A scripted manual yaw phase reduces the offset before follow is requested.
- Tracker initialization and autonomous PID commands are blocked until the
  follow command frame.
- After follow, PID yaw centers the target and PID pitch/forward changes the
  synthetic apparent target width.

Gate:

- No autonomous control samples occur before follow.
- Tracker is not initialized before follow and is initialized after follow.
- Manual handoff error is below the configured bound.
- Final center and width errors are below configured bounds.
- Manual-to-final error reduction exceeds the configured bound.
- No Gazebo, pr0p, Betaflight, screen capture, OS input, or physical RC is used.

## S12-adapter - In-Process InputAdapter Closed Loop

Goal: prove the same `FrameSource + InputAdapter` contract used by the screen
tracker loop can drive a simple game state, not only record commands.

Command:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/phase_runner.py \
  --include-adapter-dynamics \
  --duration 5 \
  --hz 20 \
  --run-id sandbox-adapter-dynamics
```

Automated status:

- `phase_runner.py --include-adapter-dynamics` adds `S12-adapter`.
- `screen_tracking_loop.run_loop()` sends yaw commands through an
  `InputAdapter`.
- The simple game frame source reads the adapter's last command and applies it
  to the next rendered frame.
- The report includes adapter command count, applied non-neutral commands,
  camera movement, final center error, and neutralization.

Gate:

- Non-neutral commands are sent through the adapter.
- Non-neutral adapter commands are applied to the game state.
- Camera movement direction matches the initial target offset.
- Final horizontal error is below the configured bound.
- Input neutralizes at the end.
- No Gazebo, pr0p, Betaflight, screen capture, OS input, or physical RC is used.

## S13-binding-dry - RC Binding Dry Sequence

Goal: prove the planned RC/channel binding sequence can be generated and
neutralized without touching OS input.

Command:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/phase_runner.py \
  --include-binding-dry \
  --run-id sandbox-binding-dry
```

Automated status:

- `phase_runner.py --include-binding-dry` adds `S13-binding-dry`.
- The gate sends yaw, pitch, roll, and throttle steps through
  `DryRunInputAdapter`.
- The report includes every step, command count, neutralization, and
  `no_os_input=true`.

Gate:

- Each planned RC axis has a dry command step.
- The adapter returns to neutral.
- No Gazebo, pr0p, Betaflight, screen capture, OS input, or physical RC is
  used.

## S14-objective - Objective Flow In One Loop

Goal: prove the project's core flow in one independent simple-game loop:
manual centering first, follow command second, tracker/PID after follow, and
adapter-applied yaw+pitch control that centers and approaches the target.

Command:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/phase_runner.py \
  --include-objective-loop \
  --duration 6 \
  --hz 20 \
  --run-id sandbox-objective-loop
```

Automated status:

- `phase_runner.py --include-objective-loop` adds `S14-objective`.
- The sim starts with a visible center offset and small apparent target width.
- A scripted manual phase sends yaw commands through an `InputAdapter`-like
  recorder before the follow event.
- The follow event is recorded as button press/release evidence.
- Tracker initialization and autonomous PID samples are blocked until follow.
- Autonomous yaw and pitch commands are sent through the same adapter and then
  applied to the next simple-game frame.

Gate:

- Manual commands occur before follow.
- No autonomous control samples occur before follow.
- The tracker is initialized only after follow.
- Non-neutral adapter commands are sent and applied.
- Final center error and final width error are below configured bounds.
- The adapter neutralizes at the end.
- No Gazebo, pr0p, Betaflight, screen capture, OS input, or physical RC is
  used.

## Reporting

Every meaningful run should produce a short summary with:

- Game/sim and capture crop.
- Tracker and PID settings.
- Input adapter.
- Verdict.
- Found ratio.
- Center error RMS/P95.
- Latency when available.
- PID saturation.
- Target-loss events.
- Approach metric status.

Automated status:

- `phase_runner.py` writes paired JSON and Markdown phase reports.
- `sandbox_acceptance_runner.py` can produce S0-S14 synthetic evidence,
  optional local simple-window evidence, and the final decision in one command.
- Acceptance reports include `isolation_status`; an isolation audit failure
  rejects the acceptance run even if tracker/PID gates pass.
- `decision_report.py` combines the latest
  S8/S9-game/S10-range/S11-handoff/S12-adapter/S13-binding-dry/S14-objective
  synthetic report and
  simple-window S7 real-screen dry-run report into
  `SIMPLE_SANDBOX_READY`, `WAITING`, or `REJECT`.
- Markdown reports include `Run Summary`, `Phase Verdicts`, `Evidence Summary`,
  and `Raw Metrics`.
- The evidence table surfaces found ratio, loss count, target motion, yaw/pitch
  command bounds, latency/forward metric, real-input state, neutralization, and
  log path.
- Reports explicitly state that the run is sandbox-only and does not require
  the Gazebo/Betaflight launcher path.
