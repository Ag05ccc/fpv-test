# Game Screen Sandbox

Isolated screen/game-in-the-loop experiment for Kenet tracker + PID work.

This directory is intentionally not part of the default Gazebo, Betaflight, or
Kenet SITL launch path. It can be removed without changing the main project.

See the root plan: `simple_game_sitl.md`.

## Current Verified State

As of 2026-07-05 on this machine:

- `ffmpeg/x11grab` real screen crop works on `DISPLAY=:1`.
- X11 window-title discovery works through `xwininfo`.
- `evdev` is installed in `fpv_env` through `requirements-input.txt`.
- `/dev/uinput` is writable by the current user.
- `simple_target_game.py` provides a local OpenCV target window for real
  screen-capture smoke without any external simulator.
- `phase_runner.py` can pass the S0-S4 core and can run/report S5 latency,
  S6 approach dry-run, S7 combined yaw+pitch dry-run, S8 synthetic
  moving-target, S9 in-process game-camera, S10 range, S11 handoff, S12
  adapter-driven, S13 binding-dry, and S14 objective-flow gates without
  touching the Gazebo/Betaflight launch path.

## First Smoke Commands

Run the current independent acceptance bundle. Add `--include-simple-window`
when an X11/OpenCV window can be opened:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/sandbox_acceptance_runner.py \
  --run-id sandbox-acceptance \
  --include-simple-window
```

Run the isolation audit by itself:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/isolation_audit.py --json
```

Run the S0-S4 synthetic phase gates and write JSON/Markdown evidence under
`logs/game_screen_sandbox/`:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/phase_runner.py \
  --duration 2 \
  --hz 10 \
  --run-id sandbox-phase
```

List visible X11 windows and resolve a game/sim crop by title:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/x11_window.py --list

fpv_env/bin/python experiments/game_screen_sandbox/x11_window.py \
  --title pr0p
```

Capture one frame for target selection. First run writes a frame and returns
`WAITING`; inspect the image and rerun with the chosen bbox:

If the requested window is not visible, this remains a clean `WAITING` result
instead of raising a traceback.

```bash
fpv_env/bin/python experiments/game_screen_sandbox/bbox_tool.py \
  --window-title pr0p \
  --capture-backend ffmpeg \
  --frame-path logs/game_screen_sandbox/pr0p-target-frame.png

fpv_env/bin/python experiments/game_screen_sandbox/bbox_tool.py \
  --window-title pr0p \
  --capture-backend ffmpeg \
  --bbox 100,100,200,120 \
  --frame-path logs/game_screen_sandbox/pr0p-target-frame.png \
  --report-path logs/game_screen_sandbox/pr0p-target-bbox.json
```

When the simulator window is intentionally visible, you can also select the
target in an OpenCV ROI UI. This still sends no input to the simulator:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/bbox_tool.py \
  --window-title pr0p \
  --capture-backend ffmpeg \
  --interactive-select \
  --frame-path logs/game_screen_sandbox/pr0p-target-bbox-frame.png \
  --report-path logs/game_screen_sandbox/pr0p-target-bbox.json
```

Run the external window preflight after choosing a bbox. This is the first
one-command check for a pr0p/SITL Forge/offline FPV simulator window:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/sandbox_status_report.py \
  --window-title pr0p \
  --bbox-file logs/game_screen_sandbox/pr0p-target-bbox.json
```

This command is status-only. It does not launch a simulator, capture frames, or
send `uinput`; it reports the next missing setup gate from existing evidence.
Add `--require-live-input` when you want the same status command to require the
latest `external_live_input_readiness.py` report before returning `READY`.
Matching evidence must also use the current bbox from `--bbox-file`; reports
from an older target selection are ignored and the status remains `WAITING`.
The JSON/Markdown output includes `resume_commands`. Each command records
`safety_class`, `expected_status`, and `unblocks`; commands that can send
virtual RC are marked with `sends_uinput: true` and require explicit ack flags.
Commands may also include `requires_pass`, a list of status items that must be
`PASS` before the resume runner may execute the command.
If you pass `--axis-expected-shifts` and `--axis-min-shift-px`, those signed
axis expectations are preserved in the generated status, live-input readiness,
and bounded live-follow resume commands.
The JSON/Markdown output also includes `evidence_freshness`. Add
`--require-fresh-evidence` when a stale or missing required evidence file should
block `READY`; by default this freshness check is advisory only.

Use the operator preflight when you want one status-only handoff view before
touching the simulator. It reports the current readiness stage, first operator
action, command state counts, visible X11 window candidates, and the
recommended command without launching capture or sending input:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/external_operator_preflight.py \
  --window-title pr0p \
  --bbox-file logs/game_screen_sandbox/pr0p-target-bbox.json \
  --require-live-follow \
  --require-fresh-evidence \
  --evidence-stale-after-s 3600 \
  --axis-expected-shifts yaw:+x,pitch:-y \
  --axis-min-shift-px 1 \
  --max-window-candidates 5
```

When the requested title does not match, each `window_discovery` candidate also
includes exact `rerun_operator_preflight_command` and
`capture_bbox_frame_command` strings for that candidate title, plus
`capture_bbox_frame_region_command` for title-independent region capture and
`dry_run_sequence_region_command` for a title-independent dry-run attempt after
the bbox file exists.
The report also writes `candidate_action_plan` and
`recommended_candidate_action`; these pick the next safe non-`uinput` step for
each visible candidate. If the bbox is missing, the next step is region frame
capture. If the bbox is ready but dry evidence is missing, the next step is
region dry-run sequence. The guard remains manual: run a candidate command only
after confirming it is the intended simulator/FPV view.
Tooling/editor/shell windows such as VS Code or `user@host: ~` terminals are
reported under `excluded_candidates` instead of `candidate_action_plan`, even
when their title contains the requested simulator text.

Use the candidate action runner when you want that candidate plan as a guarded
handoff. With no `--candidate-index`, it stays plan-only and does not capture or
send input:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/external_candidate_action_runner.py \
  --window-title pr0p \
  --bbox-file logs/game_screen_sandbox/pr0p-target-bbox.json \
  --require-live-follow \
  --require-fresh-evidence \
  --evidence-stale-after-s 3600 \
  --axis-expected-shifts yaw:+x,pitch:-y \
  --axis-min-shift-px 1 \
  --max-window-candidates 5
```

After confirming the candidate is the simulator/FPV view, run exactly one safe
non-`uinput` candidate action. Prefer `--candidate-window-id` from the latest
candidate plan when possible; `--candidate-index` is also supported but can
change when the visible window order changes:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/external_candidate_action_runner.py \
  --window-title pr0p \
  --bbox-file logs/game_screen_sandbox/pr0p-target-bbox.json \
  --require-live-follow \
  --require-fresh-evidence \
  --evidence-stale-after-s 3600 \
  --axis-expected-shifts yaw:+x,pitch:-y \
  --axis-min-shift-px 1 \
  --max-window-candidates 5 \
  --candidate-window-id 0xWINDOW_ID \
  --ack-candidate \
  --execute-safe
```

The runner can also select with `--candidate-title` for an exact title or
`--candidate-title-contains` for a unique substring. Ambiguous substring matches
are rejected. Obvious tooling/editor windows such as VS Code are also rejected
even if their title contains the requested simulator text; prefer
`--candidate-window-id` from the latest candidate plan after visually confirming
the window is the simulator/FPV view.

After inspecting the captured frame and choosing the bbox, keep the same
candidate selector and add `--candidate-bbox X,Y,W,H`. This writes the bbox file
through the selected candidate's region crop, without relying on the window
title. The bbox coordinates are relative to the selected candidate region; if
the bbox falls outside that region, the runner rejects it before running any
capture/write command:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/external_candidate_action_runner.py \
  --window-title pr0p \
  --bbox-file logs/game_screen_sandbox/pr0p-target-bbox.json \
  --require-live-follow \
  --require-fresh-evidence \
  --evidence-stale-after-s 3600 \
  --axis-expected-shifts yaw:+x,pitch:-y \
  --axis-min-shift-px 1 \
  --max-window-candidates 5 \
  --candidate-window-id 0xWINDOW_ID \
  --candidate-bbox X,Y,W,H \
  --ack-candidate \
  --execute-safe
```

When a candidate action actually runs, the runner writes a second
`post_operator_preflight` report so the same artifact shows whether the selected
region capture or dry-run changed the next readiness gate.

Use the external target acceptance command for the final status-only answer on
a real pr0p/SITL Forge/offline FPV window. It defaults to the project target:
fresh bounded live-follow evidence. It does not launch capture or send input,
records a Gazebo/Betaflight process-boundary audit, and writes an
`acceptance_decision` summary with the first blocking stage and next operator
action. It also surfaces an `operator_command_queue` with safety metadata for
the next available commands, including `command_state` and `unmet_requires_pass`.
`operator_command_summary` gives the counts by state and the first
available/blocked/ack-required command.

```bash
fpv_env/bin/python experiments/game_screen_sandbox/external_target_acceptance.py \
  --window-title pr0p \
  --bbox-file logs/game_screen_sandbox/pr0p-target-bbox.json \
  --mode live-follow \
  --axis-expected-shifts yaw:+x,pitch:-y \
  --axis-min-shift-px 1 \
  --evidence-stale-after-s 3600
```

Use the resume runner when you want the same information as an actionable
handoff report. By default it is plan-only and does not run capture or input:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/sandbox_resume_runner.py \
  --window-title pr0p \
  --bbox-file logs/game_screen_sandbox/pr0p-target-bbox.json \
  --require-live-follow \
  --axis-expected-shifts yaw:+x,pitch:-y \
  --axis-min-shift-px 1
```

After the window/bbox setup is intentional, add `--execute-safe` to run only
non-uinput, non-edit resume commands. Live virtual RC still requires both
`--execute-live-input` and `--ack-live-input`. When a command runs, the runner
also writes a post-status report so the handoff shows the updated gate state.
Commands that open a manual UI, such as `interactive_bbox_select`, are marked
`requires_user_interaction: true` and remain blocked in the runner; run them
intentionally from the terminal when the target window is visible.
Commands with unmet `requires_pass` dependencies are also blocked before any
capture or virtual RC is attempted, even if execution flags are present.
The runner also writes a compact `resume_decision` summary with the first
operator action and the first unmet dependency.
It also writes `resume_step_summary`, which counts step states, live-input
commands, executed commands, and the first planned/blocked/unmet/failing step.
The status report also writes `readiness_ladder`, an S0-S5 stage view from
local isolation through bounded live follow.
It also writes `evidence_freshness` so stale or missing artifact evidence is
visible in JSON and Markdown reports.
With `--require-fresh-evidence`, stale or missing required evidence becomes a
hard `READY` blocker and the generated status command preserves the same flag.
When dry evidence is ready but live-input response is not, `resume_commands`
also includes `rc_binding_assistant.py` for the simulator `Controls -> RC
Channels` page; it is marked `sends_uinput: true` and remains blocked without
the live-input acknowledgements. Dry-run binding reports are not accepted as RC
channel binding evidence; status requires live `uinput`, acknowledgement, and
neutralization in the matching binding report.

When the target window and bbox file are ready, run the dry-run sequence. It
chains status, preflight, and follow dry-run, but still does not send `uinput`:

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

The JSON/Markdown report includes `sequence_steps` and `live_readiness`, so a
resume agent can see whether setup, preflight, or follow was the first blocking
step before any live input is attempted.

If the title is unstable but the operator preflight has listed the right visible
window candidate, use that candidate's `dry_run_sequence_region_command`. Region
mode uses the candidate crop directly and does not block on `target_window`; the
required setup items are the bbox file and the Gazebo/Betaflight process-boundary
audit.

When the dry sequence passes and the simulator's `Controls -> RC Channels`
mapping is ready, run the explicit live-input readiness gate. It re-runs the
dry sequence first, then sends the virtual-RC axis sweep only when
`--ack-live-input` is present:

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

This writes `EXTERNAL_LIVE_INPUT_READY` only when dry sequence and visual
axis-response both pass. Without `--ack-live-input`, it writes `WAITING` and
does not send virtual RC.

The status gate does not trust the top-level readiness status alone. A matching
live-input report is accepted only when it embeds S5-real proof from
`UInputAdapter` with `real_input: true`, `axis_sweep: true`, and the same signed
axis expectation if `--axis-expected-shifts` was requested.

If the simulator reacts but yaw/pitch may be inverted, make the same gate check
signed visual motion before live follow:

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

The plain axis sweep only proves "the image changed." The signed check also
records `response_shift_x_px`, `response_shift_y_px`, and `direction_status`,
and fails an axis that moves opposite the expected direction.

After live-input readiness is green, use the bounded live-follow sequence
instead of calling the follow session directly. It requires `--ack-live-input`,
rechecks live-input readiness, and then runs a short `uinput` follow session:

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

This writes `EXTERNAL_LIVE_FOLLOW_COMPLETE` only when readiness and the bounded
live follow both pass.

After that, the status-only resume command can require the live-input evidence:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/sandbox_status_report.py \
  --window-title pr0p \
  --bbox-file logs/game_screen_sandbox/pr0p-target-bbox.json \
  --require-live-input \
  --axis-expected-shifts yaw:+x,pitch:-y \
  --axis-min-shift-px 1
```

After a bounded live-follow sequence, require that final evidence instead:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/sandbox_status_report.py \
  --window-title pr0p \
  --bbox-file logs/game_screen_sandbox/pr0p-target-bbox.json \
  --require-live-follow \
  --axis-expected-shifts yaw:+x,pitch:-y \
  --axis-min-shift-px 1
```

```bash
fpv_env/bin/python experiments/game_screen_sandbox/external_window_preflight.py \
  --capture-window-title pr0p \
  --capture-backend ffmpeg \
  --tracker-bbox-file logs/game_screen_sandbox/pr0p-target-bbox.json \
  --run-id pr0p-window-preflight
```

After the dry preflight is green, add the opt-in RC channel/axis binding gate.
This sends real virtual input only when `--ack-live-input` is present:

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

If S1/S2/S7 pass but S5-real waits because there is no visual response, check
the simulator/game `Controls -> RC Channels` or axis binding page before tuning
PID gains.

Run a bounded external follow dry-run after bbox selection and preflight. This
captures the external window, runs tracker + PID, writes JSON/Markdown evidence,
and sends commands only to `DryRunInputAdapter`:

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

Only after the dry-run and RC channel axis sweep are acceptable, run the short
live-input session:

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

Run the pure-Python dry smoke. It uses synthetic frames and a dry-run input
adapter, so it does not need a game, Gazebo, Betaflight, `mss`, or `evdev`.

```bash
fpv_env/bin/python experiments/game_screen_sandbox/screen_tracking_loop.py \
  --synthetic \
  --duration 2 \
  --hz 10 \
  --dry-run
```

Run the repo-local simple target window headless first. This proves the tiny
renderer and writes the initial bbox into the JSON report:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/simple_target_game.py \
  --duration 2 \
  --fps 10 \
  --static-target \
  --save-frame logs/game_screen_sandbox/simple-game-frame.png \
  --report-path logs/game_screen_sandbox/simple-game-report.json
```

Run the full local real-window smoke with one command. This opens the simple
target window, resolves the 640x480 X11 content region, captures it through
ffmpeg/x11grab, and runs the S7 combined dry-run gate:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/simple_window_smoke.py \
  --run-id sandbox-simple-window
```

After a synthetic moving-target run and a simple-window smoke, create the
readiness decision for this isolated sandbox:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/phase_runner.py \
  --duration 6 \
  --hz 20 \
  --include-moving-target \
  --include-game-dynamics \
  --include-range-dynamics \
  --include-handoff-dynamics \
  --include-adapter-dynamics \
  --run-id sandbox-moving-target

fpv_env/bin/python experiments/game_screen_sandbox/decision_report.py \
  --run-id sandbox-decision
```

The acceptance runner above automates this synthetic phase run plus decision
step, and can also run the repo-local simple-window smoke when
`--include-simple-window` is supplied. Its report includes `isolation_status`
and rejects the run if a Gazebo/Betaflight import or newly launched forbidden
process is detected.

`SIMPLE_SANDBOX_READY` means the independent game/screen sandbox is ready for
tracker/PID dry-run development and that PID output has been applied to the
in-process simple game camera, synthetic target range, and manual-to-autonomous
handoff flow; it also proves the same `InputAdapter` contract can drive the
simple game state. The S9/S10/S11 reports also expose
`yaw_initial_command_corrective` and, for range/handoff,
`pitch_initial_command_corrective`, so an axis-sign mistake fails before a
live run hides it behind noisy motion. It does not promote pr0p, SimITL,
Betaflight, Gazebo, or live RC control.

Open the same target window, then capture it by title from another
terminal:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/simple_target_game.py \
  --show \
  --duration 60 \
  --fps 30 \
  --static-target \
  --report-path logs/game_screen_sandbox/simple-game-window-report.json

fpv_env/bin/python experiments/game_screen_sandbox/x11_window.py \
  --title "Kenet Simple Target Game" \
  --preferred-size 640x480

fpv_env/bin/python experiments/game_screen_sandbox/phase_runner.py \
  --capture-region <left,top,640,480-from-x11-window> \
  --capture-backend ffmpeg \
  --tracker-bbox <x,y,w,h-from-simple-game-report-first_bbox> \
  --include-real-combined \
  --duration 2 \
  --hz 10 \
  --run-id sandbox-simple-game-window
```

Optional real-window capture requires the `mss` package and an X11/desktop
session. On X11, `ffmpeg` can be used without installing `mss`:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/capture_window.py \
  --region 0,0,640,480 \
  --backend ffmpeg \
  --duration 5 \
  --save-frame logs/game_screen_sandbox/sample-capture.png
```

The phase runner can include the same real capture gate after a game/sim window
is visible and its crop is known:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/phase_runner.py \
  --capture-region 0,0,640,480 \
  --capture-backend ffmpeg \
  --duration 2 \
  --hz 10
```

The same gate can resolve the crop from a window title:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/phase_runner.py \
  --capture-window-title pr0p \
  --capture-backend ffmpeg \
  --duration 2 \
  --hz 10
```

After selecting a target bbox inside the captured frame, verify that the real
window frames reach the tracker:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/phase_runner.py \
  --capture-window-title pr0p \
  --capture-backend ffmpeg \
  --tracker-bbox-file logs/game_screen_sandbox/pr0p-target-bbox.json \
  --duration 2 \
  --hz 10 \
  --run-id sandbox-pr0p-tracker
```

Then run the first safe real-window visual-servo gate. This still uses dry-run
input; it proves tracker -> PID -> bounded command without steering the game:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/phase_runner.py \
  --capture-window-title pr0p \
  --capture-backend ffmpeg \
  --tracker-bbox-file logs/game_screen_sandbox/pr0p-target-bbox.json \
  --include-real-loop \
  --duration 2 \
  --hz 10 \
  --run-id sandbox-pr0p-loop-dry
```

Only after confirming a neutral/stop plan, run the shortest live uinput gate.
This sends real virtual joystick commands and therefore requires explicit
acknowledgement:

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

Measure visual response latency after a small command. Dry-run is useful for
checking the measurement path; real input again requires acknowledgement:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/latency_probe.py \
  --window-title pr0p \
  --capture-backend ffmpeg \
  --yaw 0.05 \
  --fps 10 \
  --report-path logs/game_screen_sandbox/pr0p-latency-dry.json

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

Run an RC channel/axis response sweep after opening the simulator and confirming
a neutral/stop plan:

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

If real `uinput` is available but this latency gate cannot see a visual
response, check the simulator/game `Controls -> RC Channels` or axis binding
page before changing tracker/PID code.

If visual response exists but axis direction is suspect, add a signed shift
expectation. Example: `yaw:+x` means a positive yaw command should move the
captured image in +x by at least `--latency-axis-min-shift-px` pixels:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/phase_runner.py \
  --capture-window-title pr0p \
  --capture-backend ffmpeg \
  --include-latency-probe \
  --latency-axis-sweep \
  --latency-axes yaw,pitch \
  --latency-axis-expected-shifts yaw:+x,pitch:-y \
  --latency-axis-min-shift-px 1 \
  --latency-uinput \
  --ack-live-input \
  --duration 1 \
  --hz 10 \
  --run-id sandbox-pr0p-axis-direction
```

Run the first approach/pitch gate after selecting a bbox. This is still dry-run:
it proves bbox width -> forward PID -> bounded pitch command without steering
the game:

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

Run the first combined yaw+pitch gate after S4 and S6 are acceptable. This is
also dry-run; it proves both controller axes stay bounded in one loop:

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

Run the pre-game moving-target gate. This uses a deterministic synthetic target
and dry-run input, so it is a regression check before a real simulator target
is selected:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/phase_runner.py \
  --include-moving-target \
  --duration 2 \
  --hz 10 \
  --min-moving-target-motion 30 \
  --run-id sandbox-moving-target
```

Each `phase_runner.py` run writes paired JSON/Markdown reports under
`logs/game_screen_sandbox/`. The Markdown report includes `Run Summary`,
`Phase Verdicts`, `Evidence Summary`, and `Raw Metrics`, with an explicit
sandbox-only isolation note.

Virtual joystick output is deliberately optional. The first implementation uses
`DryRunInputAdapter`; real `uinput` support should be enabled only after a
manual neutral/kill-switch smoke passes.

Real virtual joystick prerequisites can be checked without touching the game:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/virtual_input.py \
  --probe-uinput
```

If the probe reports missing `evdev`, install only the sandbox input extra:

```bash
fpv_env/bin/python -m pip install -r \
  experiments/game_screen_sandbox/requirements-input.txt
```

After confirming a neutral/stop plan, run a short real `uinput` smoke:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/phase_runner.py \
  --include-real-input \
  --uinput-smoke \
  --input-yaw 0.05 \
  --input-hold-seconds 0 \
  --run-id sandbox-uinput-smoke
```

To bind the simulator controls, open the simulator/game
`Controls -> RC Channels` page, select one axis binding, and pulse that axis
with the assistant:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/rc_binding_assistant.py \
  --window-title pr0p \
  --tracker-bbox-file logs/game_screen_sandbox/pr0p-target-bbox.json \
  --axes yaw \
  --axis-value 0.6 \
  --hold-seconds 0.8 \
  --neutral-seconds 0.3 \
  --uinput \
  --ack-live-input \
  --run-id pr0p-bind-yaw
```

Repeat with `--axes pitch`, `--axes roll`, and `--axes throttle` as needed.
The assistant reports the virtual joystick hints used by the sandbox:
roll=`ABS_X`, pitch=`ABS_Y inverted`, yaw=`ABS_RX`, throttle=`ABS_RY inverted`.
