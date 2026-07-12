# SimITL / pr0p Probe

Isolated pr0p + SimITL spike. This does not change the default Gazebo,
Betaflight, Kenet, or acceptance paths.

See `PLAN.md` for the staged probe.

## Core Goal Status

Use this first when the question is whether the alternative sim path is moving
toward the real goal:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_core_goal_report.py \
  --bbox x,y,w,h \
  --run-id pr0p-core-goal
```

If the target selection was saved by `pr0p_bbox_tool.py`, pass it directly with
`--bbox-file logs/simitl_pr0p/pr0p-target-bbox.json` instead of copying the
numbers by hand.

It reads existing evidence only and reports three outcomes:

- `rc_manual_flight`: pr0p local race, RC input, AUX1/CH5 arm, and bounded yaw/pitch/roll response.
- `camera_tracker`: FPV capture, bbox, tracker/PID dry-run, and tracking evidence.
- `autopilot_control`: signed yaw/pitch response plus live tracking-control evidence.

Read `completion_status` when asking whether the full independent sim goal is
actually finished. `status=PASS` only means the three core gates above are
proven. `completion_status=COMPLETE` is reserved for the full sequence through
extended follow, airborne static-target yaw, pitch/approach, and real
manual-to-autonomous handoff. Until then it remains `IN_PROGRESS` and
`completion_missing` lists the unfinished gates.

The same JSON includes `objective_requirements` plus compact
`objective_requirement_statuses`, `objective_requirement_missing`, and
`objective_requirement_failed` fields. Use that matrix when checking the
user-facing requirements directly: `gazebo_independent_path`,
`target_bbox_selected`, `manual_rc_flight`, `camera_image_to_tracker`,
`autopilot_pid_control`, `extended_closed_loop_follow`,
`airborne_static_yaw_centering`, `pitch_approach_control`, and
`manual_to_auto_handoff` must all be `PASS` before this path should be treated
as the measured operating flow.

If this report says `WAITING`, follow its `Next action` before chasing lower
level diagnostics. Downstream goals are dependency-gated: `camera_tracker` and
`autopilot_control` stay blocked until `rc_manual_flight` proves arm plus
bounded yaw/pitch/roll response.
The report also writes a `Commands` section with the current plan/live commands
for RC/manual flight, acceptance-chain, response, and tracking gates.
Use `Next command key` for the single recommended command at the current gate.
After the three core goals are already `PASS`, the recommended key becomes
`extended_follow_live`, a longer bounded closed-loop follow run that calls
`pr0p_tracking_acceptance_runner.py` directly with the proven arm-first hover
profile. The signed response and RC/manual prerequisites must already be PASS.
This is reported as a separate `post_core_gates.extended_follow` gate, so
`Core Goal Status = PASS` does not by itself unlock the post-core target phase.
Move on only after `extended_follow` is also `PASS`; a short tracking acceptance
run remains valid core evidence but leaves this post-core gate `WAITING`.
When `extended_follow` is `PASS`, the next key becomes `moving_target_yaw_plan`.
That command runs a safe synthetic yaw-only regression first and then leaves the
real pr0p airborne static-target gate `WAITING` until a static target is
selected from the FPV view and the vehicle is flown while pitch/approach stay
disabled.

Use the guarded acceptance runner to capture the real airborne static-target yaw
tracking report. It is plan-only by default and will not send OS input without
both `--run-live-gate` and the live acknowledgements:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_moving_target_acceptance_runner.py \
  --bbox-file logs/simitl_pr0p/pr0p-target-bbox.json \
  --run-live-gate \
  --ack-live-input \
  --ack-airborne-static-target \
  --run-id pr0p-moving-target-acceptance
```

After that command writes or identifies a live `*-tracking-acceptance.json`,
refresh the moving-target report with that file:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_moving_target_yaw_plan.py \
  --bbox-file logs/simitl_pr0p/pr0p-target-bbox.json \
  --execute-synthetic \
  --real-tracking-report path/to/*-tracking-acceptance.json \
  --ack-airborne-static-target \
  --run-id pr0p-moving-target-yaw
```

When the guarded acceptance report is `PASS`, `pr0p_core_goal_report.py`
selects `airborne_static_yaw_refresh` as the next key. That is a dry
`pr0p_moving_target_yaw_plan.py --real-tracking-report ...` refresh, so
`pr0p_core_next_runner.py --execute-next` can run it without
`--ack-live-command`.

The real gate requires yaw-only live input, `duration >= 30 s`,
`found_ratio >= 0.90`, no more than 2 loss events, image-space motion of at
least 30 px, bounded yaw, zero pitch output, loop-end FC still armed, no
screen-fixed lock suspicion, and measured center-error convergence
(`horizontal_error_reduction_ratio >= 0.50`, final center error <= 40 px). The
explicit `--ack-airborne-static-target` flag records that this is the first
static-target airborne phase; real moving targets or ghosts are a later phase.

When `moving_target_yaw` is `PASS`, the next key becomes `approach_pitch_plan`.
That gate starts with a synthetic range-dynamics regression that proves pitch
can reduce bbox width error, then waits for real pr0p approach evidence:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_approach_pitch_plan.py \
  --bbox-file logs/simitl_pr0p/pr0p-target-bbox.json \
  --execute-synthetic \
  --real-tracking-report path/to/*-tracking-acceptance.json \
  --ack-real-approach-target \
  --run-id pr0p-approach-pitch
```

After the synthetic approach report is `PASS` and real approach evidence is
still missing, the current key becomes `approach_pitch_live`. That guarded live
command runs `pr0p_tracking_acceptance_runner.py` with `--enable-pitch`, the
arm-first hover profile, and bounded yaw/pitch thresholds, so
`pr0p_core_next_runner.py --execute-next` also requires `--ack-live-command`.
When that pitch-enabled tracking acceptance is `PASS`, the current key becomes
`approach_pitch_refresh`, a dry `pr0p_approach_pitch_plan.py
--real-tracking-report ... --ack-real-approach-target` refresh.

The real approach gate requires live `--enable-pitch`, `duration >= 20 s`,
`found_ratio >= 0.90`, no more than 2 loss events, bounded pitch, nonzero pitch
usage, and measured bbox-width error reduction. This keeps bbox width as a
measured range signal, not a free pass: the initial width error, final width
error, and reduction ratio must all meet the gate.

When `approach_pitch` is `PASS`, the next key becomes `handoff_plan`. This gate
is the actual operating-flow proof: the user manually centers/selects the
target, sends the follow command, the tracker initializes only after that
command, and autonomous yaw/pitch control takes over without pre-handoff control
samples.

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_handoff_acceptance_runner.py \
  --bbox-file logs/simitl_pr0p/pr0p-target-bbox.json \
  --run-live-gate \
  --ack-live-input \
  --ack-real-handoff \
  --run-id pr0p-real-handoff
```

After the synthetic handoff report is `PASS` and real handoff evidence is still
missing, the current key becomes `handoff_live`. That key runs the guarded
`pr0p_handoff_acceptance_runner.py` command, so `pr0p_core_next_runner.py
--execute-next` also requires `--ack-live-command`. That command is plan-only
unless `--run-live-gate` is present, and it refuses to send OS input without
both `--ack-live-input` and `--ack-real-handoff`. Its JSON output is the real
handoff report consumed by the gate refresh. When that report is `PASS`, the
current key becomes `handoff_refresh`, a dry canonical refresh:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_handoff_plan.py \
  --bbox-file logs/simitl_pr0p/pr0p-target-bbox.json \
  --execute-synthetic \
  --real-handoff-report path/to/*-real-handoff.json \
  --ack-real-handoff \
  --run-id pr0p-handoff
```

The synthetic part runs in-process and sends no OS input. The real handoff
acceptance runner converts the follow-time tracking probe into the PASS/FAIL
report. The gate then requires `follow_command_sent`, no auto-control samples
before follow, tracker initialization after follow, bounded yaw/pitch, stable
tracking, and measured center/width error reduction. `--ack-real-handoff`
records that the report came from a real manual-to-autonomous pr0p handoff.

To plan or safely execute that current command, use the core next runner:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_core_next_runner.py \
  --bbox x,y,w,h \
  --run-id pr0p-core-next
```

`pr0p_core_next_runner.py` also accepts the same `--bbox-file ...` argument and
will expand it into bound `--bbox` values in the generated next command.
Add `--allow-physical-mapping` when the next RC/manual-flight command should
accept the physical transmitter mapping instead of requiring the virtual input
profile.

It is plan-only by default. `--execute-next` runs the selected command only
when requested; if the command is live, it also requires `--ack-live-command`.
Execution is blocked if the selected command still contains placeholder bbox
arguments such as `x,y,w,h`; choose a real target bbox first.
Every core-next run also writes an isolation precheck report. The JSON exposes
`isolation_status`, `isolation_report`, and `isolation_violation_count`; if the
status is not `PASS`, core-next returns `GAZEBO_INDEPENDENCE_FAILED` and does
not run either `--execute-next` or `--execute-operator-next`.
Before executing any live command, the runner also performs a read-only
`independent_sim_readiness.py` precheck. If the combined independent readiness
is not `INDEPENDENT_SIM_READY`, the live command is not started even when
`--ack-live-command` is present.
To run that same check without executing anything, add
`--precheck-live-readiness` to the plan command:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_core_next_runner.py \
  --bbox-file logs/simitl_pr0p/pr0p-target-bbox.json \
  --precheck-live-readiness \
  --run-id pr0p-core-next-precheck
```

The resulting JSON includes `live_precheck_status`,
`safe_to_execute_live_with_ack`, and `safe_to_execute_live_blockers` so a script
or operator can tell whether the live command is ready before adding
`--execute-next --ack-live-command`.
It also includes `next_objective_gate` plus a
`next_objective_gate_snapshot`, making the selected command traceable to the
specific core or post-core gate it is meant to advance.
For the live operator side, add `--precheck-operator-readiness`. That status-only
check writes `operator_precheck_status`,
`safe_to_execute_live_with_operator_precheck`, and
`safe_to_execute_live_operator_blockers` into the same core-next JSON. It checks
the pr0p window/bbox/dry-run/live-follow evidence without sending uinput.
The same JSON also exposes `operator_readiness_stage`,
`operator_readiness_command_gate`,
`operator_readiness_next_action`,
`operator_readiness_recommended_command`,
`operator_readiness_recommended_candidate_action`,
`operator_readiness_freshness_gate_status`,
`operator_readiness_stale_evidence`, and
`operator_readiness_missing_evidence` so stale/missing operator evidence is
visible without opening the nested preflight report. The recommended command
fields are status-only guidance; live execution still requires the combined
precheck and explicit live ack.
`operator_readiness_next_command_packet` is the copy-run packet for the next
safe operator-side command. When a real external window candidate is available
it prefers that candidate's region command; otherwise it falls back to the
recommended command from the operator status queue. `live_blocker_summary`
also repeats this as `operator_next_command`, with its source, safety class,
uinput flag, unmet prerequisites, and guard text.
The flattened window-discovery fields
`operator_readiness_window_discovery_status`,
`operator_readiness_window_discovery_reason`,
`operator_readiness_window_title_candidates`, and
`operator_readiness_excluded_window_candidates` show whether the pr0p/FPV
window was found and which tooling/editor/shell candidates were rejected.
`live_blocker_summary` repeats the compact view as
`operator_window_title_candidates` and `operator_excluded_window_candidates`.
To let core-next run that operator-side command, use `--execute-operator-next`.
That path is intentionally narrower than `--execute-next`: it refuses commands
that send uinput, contain live markers, still have unmet prerequisites, or keep
placeholder arguments. If the command came from an external-window candidate,
add `--ack-operator-candidate` only after confirming that candidate is the
intended FPV/sim view. This can refresh dry/capture evidence without opening
the live `airborne_static_yaw_live` gate.
After a guarded operator command runs, core-next immediately reruns the
status-only operator precheck and records `post_operator_readiness_summary`,
`post_operator_precheck_status`, `post_operator_readiness_stage`,
`post_operator_missing_evidence`, `post_operator_stale_evidence`, and
`post_operator_window_discovery_status`. Use those fields to decide whether the
dry/capture evidence actually advanced the chain before attempting live input.
If `--operator-bbox-file` is supplied, its bbox must match the command target
bbox from `--bbox`/`--bbox-file`; mismatches are reported as
`bbox_file_mismatch` and block live execution.
Automation should prefer `safe_to_execute_live_with_all_prechecks` and
`safe_to_execute_live_required_blockers` for the final live/no-live decision,
because those fields combine the independent sim readiness and operator/UI
readiness blockers.
`live_blocker_summary` is the compact handoff field for humans and scripts: it
ties the current `next_objective_gate` to the independent readiness state,
operator readiness state, missing/stale operator evidence, and the next safe
operator command recommendation.
Live execution runs that operator readiness check by default before starting the
selected command; `--skip-operator-readiness-precheck` is the explicit override.

After `moving_target_yaw_plan` has produced synthetic PASS but the real
airborne static-target evidence is still missing, the current key becomes
`airborne_static_yaw_live`. That key runs the guarded
`pr0p_moving_target_acceptance_runner.py` live command and therefore requires
both `--execute-next` and `--ack-live-command` in the core-next runner.
After that live acceptance is `PASS`, the current key becomes
`airborne_static_yaw_refresh`, which is a dry yaw-plan refresh using the
captured tracking report.

For the current first gate, use the RC/manual-flight runner:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_rc_manual_flight_runner.py \
  --allow-physical-mapping \
  --bbox x,y,w,h \
  --run-id pr0p-rc-manual-flight
```

Use `--allow-physical-mapping` when the primary RC axes are intentionally bound
to the physical transmitter in `Controls -> RC Channels`. Omit it when this run
is meant to prove the Kenet virtual input/autopilot command path.

It is plan-only by default. Live mode is the first real sim-control attempt:
after AUX1/CH5 arm acceptance, the same gate checks bounded yaw, pitch, and
roll response. Live mode requires explicit launch/UI/input acknowledgements.

## Combined Readiness

Read the latest game-screen and pr0p evidence without launching anything:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/independent_sim_readiness.py \
  --bbox-file logs/simitl_pr0p/pr0p-target-bbox.json \
  --run-id independent-sim
```

This report should stay `WAITING` until the game-screen sandbox is ready and
the live pr0p websocket, capture, input, response, and tracking gates are
proven. Evidence older than 24 hours is treated as stale by default; use
`--max-evidence-age-s` only when replaying older logs deliberately.
The game-screen decision must also declare and pass the current required phase
set, including `S13-binding-dry`. Older `SIMPLE_SANDBOX_READY` files that do not
carry the current phase schema are reported as `MISMATCH` and must be
regenerated with `sandbox_acceptance_runner.py`.
If P1 install-discovery is still `WAITING`, the report must point to the
isolated install/download step before it suggests starting pr0p or entering a
local race.
If `P1-client-executable` is still `WAITING`, the report must stop at the
manual updater/install step even when the updater download itself is present.
When this report reaches `INDEPENDENT_SIM_READY`, continue with
`pr0p_core_goal_report.py` or `pr0p_core_next_runner.py`; those commands own the
measured extended-follow, airborne static yaw, approach/pitch, and handoff
sequence.
`INDEPENDENT_SIM_READY` means the isolated game-screen and pr0p evidence paths
are promotable, not that the full autonomous-follow objective is complete. Read
`objective_completion_status` in this readiness report for that. It mirrors the
core-goal completion state and stays `IN_PROGRESS` with
`objective_completion_missing` until the post-core yaw, approach, and handoff
gates are proven.

## Safe Suite

Run the safe P0-P6 summary at any time:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_probe_suite.py \
  --run-id pr0p-suite
```

The suite never sends real OS input. Before pr0p is open, an overall `WAITING`
verdict is expected and the report shows which phase is waiting. After selecting
a target, pass `--tracking-bbox x,y,w,h` to include the P6 tracker/PID dry-run.
The `P0-isolation` gate scans the isolated Python runtime code and fails if it
imports or commands the Gazebo/SITL launcher path.
P1 is intentionally split: `P1-install-discovery` proves the updater/download
state, while `P1-client-executable` proves a runnable pr0p client exists in the
isolated root.
The P4 config patch check is dry-run only; it reports whether `input.json` can
be patched into the expected virtual profile without editing the file.
The `P6-synthetic-e2e-dry-run` gate is sandbox-only and should pass even before
pr0p is open; it proves the baseline capture -> tracker -> PID -> dry input
loop without Gazebo or a simulator window. Its S4 JSONL log is checked by
`P6-synthetic-log-check`.

Create a live-run manifest from the latest suite report:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_live_run_manifest.py \
  --run-id pr0p-live
```

The manifest lists the exact next command, the evidence file used for each
phase, and which commands would send live input if executed.

## P0 Preflight

First verify this spike is still isolated from Gazebo runtime paths:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_isolation_check.py \
  --run-id pr0p-isolation
```

Run this before downloading or starting pr0p:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/preflight_probe.py \
  --install-root /tmp/fpv-test-simitl-pr0p \
  --run-id pr0p-preflight
```

Expected before pr0p is running: overall `WAITING` is normal if port `5761` is
closed. Display/capture, virtual input, and install root should be `PASS`.

## P1 Manual pr0p Smoke

Discover the current Linux updater without downloading:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_install_probe.py \
  --install-root /tmp/fpv-test-simitl-pr0p \
  --run-id pr0p-install
```

Download the updater into the isolated install root:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_install_probe.py \
  --install-root /tmp/fpv-test-simitl-pr0p \
  --download \
  --run-id pr0p-install-download
```

This only downloads the updater and records size/hash metadata; it does not
execute it. Before executing the downloaded updater, generate a bounded launch
plan:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_updater_runner.py \
  --install-root /tmp/fpv-test-simitl-pr0p \
  --run-id pr0p-updater-dry
```

To launch the updater from the isolated root through the guarded runner:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_updater_runner.py \
  --install-root /tmp/fpv-test-simitl-pr0p \
  --launch-updater \
  --ack-external-binary \
  --leave-running \
  --ack-leave-running \
  --run-id pr0p-updater-launch
```

You can also run `/tmp/fpv-test-simitl-pr0p/updater` manually. After the updater
finishes, verify that a client executable exists:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_client_probe.py \
  --install-root /tmp/fpv-test-simitl-pr0p \
  --run-id pr0p-client
```

After the client is installed/running, start a local race and connect
Betaflight Configurator/web configurator to:

```text
ws://127.0.0.1:5761
```

Do not commit downloaded binaries or extracted game files.

Optional UI dry-run for the known local time-attack path:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_ui_smoke.py \
  --run-id pr0p-ui-dry
```

To actually click through `skip -> Local -> Time attack -> quad -> scene ->
track`, run:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_ui_smoke.py \
  --send-ui \
  --ack-live-ui \
  --run-id pr0p-ui-live
```

This only navigates pr0p menus. It does not prove the virtual FC opened; rerun
P2/P2-msp-readonly immediately after it.

Bounded live-session runner:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_live_session_runner.py \
  --run-id pr0p-live-session-dry
```

The dry-run only checks the isolated executable path and the current
`Controls -> RC Channels` mapping. It does not launch pr0p.

When pr0p should be launched for a bounded measurement, use:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_live_session_runner.py \
  --launch-pr0p \
  --ack-live-launch \
  --send-ui \
  --ack-live-ui \
  --run-suite \
  --run-id pr0p-live-session
```

This starts pr0p from `/tmp/fpv-test-simitl-pr0p`, optionally clicks into the
known local race path, runs the safe suite while pr0p is alive, then terminates
the process it started and reports any leftover pr0p process lines. If the RC
mapping is still bound to a physical transmitter, the runner reports
`PR0P_INPUT_MAPPING_NOT_READY`; fix it from `Controls -> RC Channels` or the
backup-backed config patch before trusting P5/P6 live control.

For the runtime input-source check, create the virtual RC device before pr0p is
launched and keep the same device open while measuring visual response:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_live_session_runner.py \
  --launch-pr0p \
  --ack-live-launch \
  --send-ui \
  --ack-live-ui \
  --hold-uinput \
  --ack-live-input \
  --run-live-response \
  --response-axis throttle \
  --response-magnitude 0.35 \
  --response-image-axis y \
  --response-expected-sign 0 \
  --run-id pr0p-live-persistent-uinput
```

If this remains `WAITING`, do not trust tracker/PID live-control claims yet.
It means pr0p was launched with the virtual RC device present, but the runtime
still did not produce a measurable vehicle/view response.

## P2 Websocket Probe

After a local race is running:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/msp_ws_probe.py \
  --host 127.0.0.1 \
  --port 5761 \
  --run-id pr0p-ws
```

This checks only the Configurator-style websocket handshake. Then run the
read-only MSP semantic probe:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/msp_ws_semantic_probe.py \
  --host 127.0.0.1 \
  --port 5761 \
  --run-id pr0p-msp-ws
```

This sends only `MSP_API_VERSION`. It does not arm, throttle, or write RC
channels. A `PASS` here means the websocket is actually carrying MSP bytes, not
just accepting a browser-style connection.

Then read the FC status without sending commands:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/msp_ws_status_probe.py \
  --host 127.0.0.1 \
  --port 5761 \
  --run-id pr0p-fc-status
```

This reports `armed`, active modes, and arming-disable flags when Betaflight
`MSP_STATUS_EX` is available. A `PASS` means status is readable; notes such as
`FC_NOT_ARMED` or `ARMING_DISABLED:THROTTLE` are diagnostic, not RC writes.
Use multiple samples to separate transient boot/calibration flags from persistent
arm blockers:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/msp_ws_status_probe.py \
  --samples 6 \
  --interval 1 \
  --run-id pr0p-fc-status-monitor
```

Latest live finding: `logs/simitl_pr0p/20260707-105938-pr0p-fc-status-monitor-v1-msp-ws-status.md`
showed 6/6 readable samples, `armed=False`, and persistent `THROTTLE` only.
Earlier single-sample status also saw transient `BOOTGRACE,CALIB`, which then
cleared. The next live-control blocker is therefore throttle-low/arm-state, not
MSP status readability.

Read the Betaflight mode ranges without sending commands:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/msp_ws_mode_ranges_probe.py \
  --host 127.0.0.1 \
  --port 5761 \
  --run-id pr0p-mode-ranges
```

Latest live finding:
`logs/simitl_pr0p/20260707-114048-pr0p-live-suite-mode-ranges-v1-suite-p2-mode-ranges-msp-ws-mode-ranges.md`
shows ARM expects `AUX1 / CH5` high, range `1700-2100`.

The throttle-low path is now measured directly:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_live_session_runner.py \
  --launch-pr0p \
  --ack-live-launch \
  --send-ui \
  --ack-live-ui \
  --hold-uinput \
  --ack-live-input \
  --run-rc-effect \
  --rc-effect-axis throttle \
  --rc-effect-magnitude 1.0 \
  --rc-effect-expected-channel throttle \
  --rc-effect-expected-direction lower \
  --run-id pr0p-live-uinput-rc-effect-throttle
```

Live result: `throttle=+1.0` drove MSP_RC throttle from `1500` to `1000`
(`THROTTLE_LOW_REACHED`), while `throttle=-1.0` drove it to `2000`. A second
status-effect gate showed that holding `throttle=+1.0` clears the `THROTTLE`
arming blocker. The next blocker is ARM AUX/button routing.

The direct button route was tested with the persistent virtual controller:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_live_session_runner.py \
  --launch-pr0p \
  --ack-live-launch \
  --send-ui \
  --ack-live-ui \
  --hold-uinput \
  --ack-live-input \
  --run-arm-button-effect \
  --arm-button south \
  --arm-button east \
  --arm-throttle-magnitude 1.0 \
  --startup-wait 8 \
  --run-id pr0p-live-uinput-arm-button
```

Live result:
`logs/simitl_pr0p/20260707-113523-pr0p-live-uinput-arm-button-v1-live-session.md`
reported `NO_UINPUT_ARM_BUTTON_EFFECT`; south/east buttons did not activate
ARM mode/status while throttle was low.

Because ARM expects AUX1/CH5 high, dry-run the AUX1 input config patch:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_input_config_patch.py \
  --role aux1 \
  --run-id pr0p-input-config-aux1-dry
```

Current dry-run evidence:
`logs/simitl_pr0p/20260707-114819-pr0p-input-config-aux1-dry-v1-input-config-patch.md`
plans slot 4 from empty to `<Joystick>/Z` and writes nothing. Applying it is
backup-backed and explicit:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_input_config_patch.py \
  --role aux1 \
  --write \
  --ack-config-write \
  --run-id pr0p-input-config-aux1-write
```

Before applying that patch, the live AUX1 effect probe correctly stays
`WAITING`: `logs/simitl_pr0p/20260707-115217-pr0p-live-uinput-rc-aux1-before-patch-v2-live-session.md`
showed no measurable MSP_RC change on CH5. After applying or manually binding
AUX1, prove CH5 movement with:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_live_session_runner.py \
  --launch-pr0p \
  --ack-live-launch \
  --send-ui \
  --ack-live-ui \
  --hold-uinput \
  --ack-live-input \
  --run-rc-effect \
  --rc-effect-axis aux1 \
  --rc-effect-magnitude 1.0 \
  --rc-effect-expected-channel aux1 \
  --rc-effect-expected-direction higher \
  --startup-wait 8 \
  --run-id pr0p-live-uinput-rc-aux1-high
```

Then prove that throttle-low plus AUX1-high actually arms the FC:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_live_session_runner.py \
  --launch-pr0p \
  --ack-live-launch \
  --send-ui \
  --ack-live-ui \
  --hold-uinput \
  --ack-live-input \
  --run-aux-arm-status \
  --aux-arm-throttle-magnitude 1.0 \
  --aux-arm-aux1-magnitude 1.0 \
  --aux-arm-baseline-samples 20 \
  --aux-arm-during-samples 6 \
  --aux-arm-interval 1.0 \
  --aux-arm-settle 0.5 \
  --startup-wait 8 \
  --run-id pr0p-live-uinput-aux1-arm-status
```

This is the official `P5-uinput-aux1-arm-status` gate. It reads MSP status
only; it does not send MSP arm or RC writes. The long baseline matters: right
after the local race starts, Betaflight still reports `BOOTGRACE,CALIB`, and
raising AUX1 during that window latches the `ARM_SWITCH` blocker. The 20x1s
baseline holds throttle-low with AUX1 neutral until those blockers clear, so
the AUX1 raise is a clean low-to-high transition. Measured live 2026-07-07:
with the long baseline the FC armed 6/6 samples (`UINPUT_AUX1_ARMED`,
`ARM_MODE_ONLY_WHILE_COMMAND_HELD`), while the short-baseline variant stayed
`WAITING` with `ARM_BLOCKERS_DURING_AUX:ARM_SWITCH,BOOTGRACE,CALIB`.

To see the full AUX1 acceptance sequence without launching pr0p or writing
config, run:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_aux1_acceptance_runner.py \
  --bbox x,y,w,h \
  --run-id pr0p-aux1-acceptance-plan
```

After AUX1/CH5 is manually bound, the same runner can execute the live RC-effect
and arm-status gates with explicit acknowledgements:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_aux1_acceptance_runner.py \
  --bbox x,y,w,h \
  --execute-dry-patch \
  --run-live-gates \
  --ack-live-launch \
  --ack-live-ui \
  --ack-live-input \
  --run-id pr0p-aux1-acceptance-live
```

Before live response tests, check whether RC input is stable:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/msp_ws_rc_probe.py \
  --run-id pr0p-rc-baseline
```

If this reports `RC_INPUT_CONTENTION`, physical/controller input is moving and
response tests are not reliable. The optional low-throttle, arm-low write
loopback is:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/msp_ws_rc_probe.py \
  --write \
  --ack-live-msp-write \
  --run-id pr0p-rc-loopback
```

If the write is acknowledged but `MSP_RC` stays at baseline, the command reached
the websocket UART but did not become the active RC source. This is an
architectural property of pr0p: the sim reads a joystick receiver, so
`MSP_SET_RAW_RC` cannot take over the RC source. The decision report therefore
accepts a fully proven virtual-joystick chain (`P5-uinput-rc-effect-throttle-low`
plus `P5-uinput-rc-effect-aux1-high` plus `P5-uinput-aux1-arm-status`, all
`PASS`) as the RC source instead of requiring this loopback.

## P3 Capture Probe

After pr0p is visible and a local race is running:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_capture_probe.py \
  --window-title pr0p \
  --duration 30 \
  --fps 30 \
  --min-fps 20 \
  --run-id pr0p-capture
```

If the window title differs, pass another `--window-title`, for example
`--window-title "SITL Forge"`, or pass `--region left,top,width,height` after
manually selecting the game viewport. The probe excludes common editor,
browser, and terminal windows by default so a documentation tab or source file
title cannot accidentally count as the FPV view. If the full window includes
menus or borders, use `--crop left,top,width,height` to select only the FPV
viewport. Before pr0p is open, `WAITING` is expected and means the probe is ready
but has no window to capture.

## P4 Virtual Input Probe

Safe dry-run first:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_virtual_input_probe.py \
  --run-id pr0p-input-dry
```

Require real `uinput` readiness without sending OS input:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_virtual_input_probe.py \
  --require-uinput \
  --run-id pr0p-input-ready
```

Only after confirming a neutral/stop plan and pr0p input mapping, run the
shortest real virtual-controller smoke:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_virtual_input_probe.py \
  --uinput-smoke \
  --ack-live-input \
  --yaw 0.03 \
  --hold-seconds 0.1 \
  --run-id pr0p-input-live-smoke
```

The default command never sends real OS input. `--uinput-smoke` is blocked unless
`--ack-live-input` is present.

Check whether pr0p itself maps the expected virtual controller:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_input_config_probe.py \
  --run-id pr0p-input-config
```

Use `--allow-physical-mapping` only when the live test is intentionally driven
by the mapped physical transmitter instead of the Kenet virtual input device.
If this reports `PR0P_VIRTUAL_INPUT_MAPPING_MISSING`, open pr0p's
`Controls -> RC Channels` screen and bind roll/pitch/throttle/yaw to the device
you want the automation to drive. For virtual-control tests that should be the
`Kenet Game Sandbox` uinput device; for manual pilot tests it can be the
physical transmitter.

The config-file path can be checked without editing anything:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_input_config_patch.py \
  --run-id pr0p-input-config-patch-dry
```

If the dry-run report verifies the proposed patch, apply it with a backup:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_input_config_patch.py \
  --write \
  --ack-config-write \
  --run-id pr0p-input-config-patch-write
```

Then rerun `pr0p_input_config_probe.py`. This writes generic
`<Joystick>/...` paths because pr0p may record the Kenet uinput device as a
generic joystick.

To inspect the latest backup without editing anything:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_input_config_restore.py \
  --run-id pr0p-input-config-restore-dry
```

To restore the latest Codex backup, with a snapshot of the current config first:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_input_config_restore.py \
  --restore \
  --ack-config-restore \
  --run-id pr0p-input-config-restore-write
```

To keep the virtual controller alive and pulse the axes while binding channels,
first run the dry plan:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_rc_channel_mapping_assistant.py \
  --run-id pr0p-rc-channel-map-dry
```

For live binding, open `Controls -> RC Channels`, be ready to bind the five
roles in order, then run:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_rc_channel_mapping_assistant.py \
  --uinput \
  --ack-live-input \
  --role all \
  --run-id pr0p-rc-channel-map-live
```

For one channel at a time, replace `--role all` with `--role throttle`,
`--role yaw`, `--role aux1`, etc. AUX1 is the ARM/CH5 input used by the
acceptance chain. The command sends real OS input only with the explicit ack
flag.

When `Controls -> RC Channels` is open, use the visual runtime gate to verify
that the running pr0p UI actually changes while a virtual RC pulse is active:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_runtime_input_visual_probe.py \
  --uinput \
  --ack-live-input \
  --axis yaw \
  --magnitude 0.6 \
  --run-id pr0p-rc-channels-visual
```

If other parts of the page animate, pass `--crop left,top,width,height` around
the RC channel bars. A `PASS` here proves runtime input visibility, not vehicle
motion. P5 response gates are still required before live tracking control.

Observed pr0p path from an active local race:

1. Click the left hamburger icon.
2. Click `Controls`.
3. Use the right-side `Input visualization` region, currently:

```bash
--crop 480,110,320,190
```

This crop produced a live `PASS` with yaw input on 2026-07-07:
`logs/simitl_pr0p/20260707-103654-pr0p-rc-channels-visual-controls-yaw-v1-runtime-input-visual.md`.

Expected virtual profile:

| Role | Kenet uinput axis | Likely pr0p control path |
| --- | --- | --- |
| Roll | `ABS_X` | `Stick/x` |
| Pitch | `ABS_Y` | `Stick/y` |
| Throttle | `ABS_RY` | `RotateY` |
| Yaw | `ABS_RX` | `RotateX` |
| AUX1 / ARM CH5 | `ABS_Z` | `Z` |

If pr0p records these as generic `<Joystick>/...` paths instead of by device
name, the probe can still accept them as the Kenet virtual profile.

## P5 Command Response Probe

Dry-run first. This captures/estimates motion if a pr0p window is visible, but
it cannot pass the gate because no real input is sent:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_response_probe.py \
  --axis yaw \
  --magnitude 0.05 \
  --image-axis x \
  --expected-sign 1 \
  --run-id pr0p-response-yaw-dry
```

After P3 capture and P4 input mapping are confirmed, run the live response gate:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_response_probe.py \
  --uinput \
  --ack-live-input \
  --axis yaw \
  --magnitude 0.03 \
  --image-axis x \
  --expected-sign 1 \
  --pre-duration 0.5 \
  --post-duration 1.0 \
  --min-shift-px 2 \
  --max-shift-px 200 \
  --run-id pr0p-response-yaw-live
```

Repeat with negative yaw and pitch after the first live run proves the axis
sign. `--expected-sign 0` records direction without failing on sign; use it only
for calibration, not acceptance.

Two measured pitfalls make the plain visual pulse insufficient (2026-07-07):

1. The vehicle does not respond while disarmed. Every response command must
   arm first; the session runner does this with `--response-arm-first`, which
   holds throttle-low, waits for boot-grace/calibration, raises AUX1, verifies
   `armed` over read-only MSP, then holds `--response-arm-throttle` (default
   `-0.35`; `-0.4` is the measured slow-climb value) together with AUX1 during
   the pulse.
2. The FPV camera uptilt makes the pad and climb views sky-dominated, so the
   pixel-shift estimator reads ~0 px even while the vehicle rotates. For
   signed acceptance use `--response-measure attitude`, which reads the signed
   `MSP_ATTITUDE` delta instead of pixels. Measured live: yaw `+0.5` pulse gave
   `+190 deg` accumulated heading (sign `+1`), pitch `+0.4` gave `-40.6 deg`
   pitch angle (sign `-1`).

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_live_session_runner.py \
  --launch-pr0p --ack-live-launch \
  --send-ui --ack-live-ui \
  --hold-uinput --ack-live-input \
  --run-live-response \
  --response-axis yaw \
  --response-magnitude 0.5 \
  --response-expected-sign 1 \
  --response-arm-first \
  --response-arm-throttle -0.4 \
  --response-measure attitude \
  --startup-wait 8 \
  --run-id pr0p-attitude-yaw-live
```

For acceptance, prefer the gated yaw/pitch runner. It first checks the latest
live manifest and will not run response commands until `P5-uinput-aux1-arm-status`
is `PASS`:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_response_acceptance_runner.py \
  --run-id pr0p-response-acceptance-plan
```

After AUX1 ARM status is proven, run the signed response gates with explicit
live acknowledgements:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_response_acceptance_runner.py \
  --run-live-gates \
  --ack-live-launch \
  --ack-live-ui \
  --ack-live-input \
  --arm-first \
  --arm-throttle -0.4 \
  --measure attitude \
  --yaw-expected-sign 1 \
  --pitch-expected-sign -1 \
  --yaw-magnitude 0.5 \
  --pitch-magnitude 0.4 \
  --run-id pr0p-response-acceptance-live
```

This exact profile measured `PASS` live on 2026-07-07 ("signed yaw and pitch
response gates passed"). Without `--arm-first` the vehicle stays disarmed and
without `--measure attitude` the sky-dominated view hides the response, so
omit them only for deliberate diagnostics.

## P6 Tracker/PID Probe

Synthetic E2E regression, no pr0p window required:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/phase_runner.py \
  --duration 2.0 \
  --hz 10 \
  --min-tracker-found-ratio 0.8 \
  --min-loop-found-ratio 0.8 \
  --log-dir logs/simitl_pr0p \
  --run-id pr0p-synthetic-e2e
```

Then verify the generated S4 loop log:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_tracking_log_check.py \
  --tracking-log path/to/*-s4-loop.jsonl \
  --run-id pr0p-synthetic-log-check
```

After P3 capture shows the FPV view, select a bbox in that captured viewport and
run the dry-run tracking/PID gate:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_bbox_tool.py \
  --run-id pr0p-bbox
```

Open the saved sample frame, choose `x,y,w,h`, then validate the overlay:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_bbox_tool.py \
  --bbox x,y,w,h \
  --run-id pr0p-bbox
```

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_tracking_probe.py \
  --bbox x,y,w,h \
  --duration 5 \
  --hz 10 \
  --tracker CSRT \
  --run-id pr0p-tracking-dry
```

This verifies `capture -> tracker -> PID -> adapter` without sending real OS
input.

Verify the JSONL log independently after a dry-run:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_tracking_log_check.py \
  --tracking-log path/to/*-p6-tracking.jsonl \
  --run-id pr0p-tracking-log-check
```

This gate checks sample count, summary/sample frame agreement, found ratio,
loss events, PID command bounds, and dry-run metadata.

For acceptance, use the gated P6 runner. It checks bbox presence, the latest
manifest's P6 dry-run gate, and the latest signed response acceptance report
before allowing live tracker/PID input:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_tracking_acceptance_runner.py \
  --bbox x,y,w,h \
  --run-id pr0p-tracking-acceptance-plan
```

After `pr0p_response_acceptance_live` is `PASS`, run yaw-only live tracking.
Use `--arm-first` so the tracker/PID output steers an armed, flying vehicle
instead of ticking over on a disarmed one; the probe arms, holds hover
throttle plus AUX1 under every PID command (tracker stop returns to the hover
hold, not a mid-air disarm), and releases everything at the end:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_tracking_acceptance_runner.py \
  --bbox x,y,w,h \
  --run-live-gates \
  --ack-live-input \
  --arm-first \
  --arm-throttle -0.4 \
  --duration 4 \
  --run-id pr0p-tracking-acceptance-live
```

Measured live 2026-07-07 with bbox `390,475,140,85` (red container cluster in
the ContainerCrash pad view): `PASS` with `found_ratio 1.0`, `0` loss events,
`40` frames, max `|yaw|` command `0.167`, and `ARM_FIRST_ARMED` (FC armed, no
blockers) — the first armed closed-loop capture -> tracker -> PID -> virtual
RC -> flying vehicle evidence on this path. pr0p must already be running with
a local race; use `pr0p_bbox_tool.py` for a fresh bbox first.

The direct live probe remains useful for manual debugging:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_tracking_probe.py \
  --bbox x,y,w,h \
  --uinput \
  --ack-live-input \
  --duration 5 \
  --hz 10 \
  --run-id pr0p-tracking-live
```

Start yaw-only. Add `--enable-pitch` only after yaw stays bounded.

## P7 Live Run Manifest

After each safe suite or dedicated probe run:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_live_run_manifest.py \
  --run-id pr0p-live
```

When a target bbox is known:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_live_run_manifest.py \
  --bbox x,y,w,h \
  --yaw-expected-sign 1 \
  --pitch-expected-sign 0 \
  --run-id pr0p-live
```

Use this report as the handoff/checklist for the next live pr0p run.

## P7.5 Ordered Acceptance Chain

The core RC/manual-flight, response, and tracking acceptance runners can be
wrapped by one ordered chain. The default is plan-only and sends no real OS
input:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_acceptance_chain_runner.py \
  --bbox x,y,w,h \
  --run-id pr0p-acceptance-chain-plan
```

The chain writes ordered stage evidence under `logs/simitl_pr0p/`. Its
`--game-log-dir` option defaults to `logs/game_screen_sandbox/`; use a separate
directory for isolated test runs so stale game-screen reports cannot promote a
pr0p-only result.

Live mode still requires explicit launch, UI, and input acknowledgements. It
stops at the first missing gate, so signed response cannot run before
RC/manual flight proves AUX1/ARM plus bounded yaw/pitch/roll response, and live
tracking cannot run before signed yaw/pitch response passes:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_acceptance_chain_runner.py \
  --bbox x,y,w,h \
  --execute-dry-patch \
  --run-live-gates \
  --ack-live-launch \
  --ack-live-ui \
  --ack-live-input \
  --arm-first \
  --arm-throttle -0.4 \
  --measure attitude \
  --attitude-min-delta 10.0 \
  --run-id pr0p-acceptance-chain-live
```

The arm-first + attitude profile is the proven live profile for the current
pr0p view: it arms before response/tracking pulses and uses signed MSP_ATTITUDE
deltas so sky-dominated FPV frames do not hide yaw/pitch response.

After a short chain has already passed, rerun the same path as an extended
follow gate:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_acceptance_chain_runner.py \
  --bbox x,y,w,h \
  --execute-dry-patch \
  --run-live-gates \
  --ack-live-launch \
  --ack-live-ui \
  --ack-live-input \
  --arm-first \
  --arm-throttle -0.4 \
  --measure attitude \
  --attitude-min-delta 10.0 \
  --duration 20.0 \
  --min-found-ratio 0.95 \
  --max-loss-events 0 \
  --run-id pr0p-extended-follow-live
```

When RC/manual flight, signed response, and live tracking all pass, the same
chain refreshes the final live manifest and writes the downstream
`pr0p_decision_refresh` and `independent_sim_readiness_refresh` stages. A chain
`PASS` means the pr0p acceptance sequence passed; inspect those generated
decision/readiness reports before treating the independent sim path as promoted.
If readiness remains `WAITING`, the usual cause is missing or stale game-screen
evidence rather than a failed pr0p chain.

To inspect the latest acceptance evidence without running any gate, use the
read-only state report:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_acceptance_state_report.py \
  --bbox x,y,w,h \
  --run-id pr0p-acceptance-state
```

## P8 Decision Report

After a live manifest exists, generate or refresh the promote/wait/reject
decision. The ordered acceptance chain does this automatically after a live
tracking `PASS`; this standalone command is for manual refresh and inspection:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_decision_report.py \
  --run-id pr0p-decision
```

This report should say `WAITING` until live pr0p capture, input, response, and
tracking-control evidence all exist. It says `REJECT` if any core phase failed.
Promotion requires a real `*-live-manifest.json` evidence source; a safe suite
alone is useful for diagnosis but cannot promote the path. Independent readiness
also checks that the live manifest points at the latest suite report, so rerun
the manifest or acceptance chain after refreshing suite evidence.

## Logs

Reports are written under:

```text
logs/simitl_pr0p/
```

## Rollback

Remove:

```text
experiments/simitl_pr0p_probe/
/tmp/fpv-test-simitl-pr0p/
logs/simitl_pr0p/
```
