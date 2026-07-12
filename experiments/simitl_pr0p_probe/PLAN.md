# SimITL / pr0p Probe Plan

Date: 2026-07-04

This is an isolated spike plan. It must not change the default Gazebo,
Betaflight, Kenet, or acceptance paths unless a later explicit decision is made.

## Goal

Evaluate whether pr0p + SimITL can become a useful Betaflight-in-loop visual
tracking sandbox for Kenet:

```text
pr0p / SimITL FPV sim
  -> screen or camera frame capture
  -> Kenet tracker
  -> Kenet PID
  -> virtual RC / MSP / game input
  -> pr0p virtual Betaflight FC
```

The question is not "can this replace Gazebo today?" The question is:

1. Can it run reliably on this machine?
2. Can Kenet receive the FPV image?
3. Can Kenet send control input into the virtual FC/sim?
4. Does yaw/pitch command response avoid the Gazebo-style runaway class?
5. Can the loop be tested and logged repeatably?

## Sources Checked

- pr0p site: https://pr0p.dev/
- pr0p download/help pages: Linux client currently advertised; Configurator
  connection is `ws://127.0.0.1:5761` during a race.
- pr0p blog: describes the latency/stale-state problem in ordinary external
  SITL ping-pong loops and the SimITL/pr0p design response.
- SimITL GitHub: https://github.com/AJ92/SimITL
  - SimITL wraps Betaflight SITL.
  - README build path is `./setup.sh` then `./build.sh` from Linux/WSL.
  - It currently states Betaflight `2025.12.0` pre-release plus changes/fixes.
  - License is GPL-3.0.

## Isolation Rules

- No default launcher changes.
- No imports from main Kenet runtime into this experiment unless behind an
  explicit adapter.
- No repository-wide environment variables.
- No binaries, downloaded zips, Steam files, or pr0p install contents committed.
- Keep all runtime files under one of:
  - `/tmp/fpv-test-simitl-pr0p/`
  - `/home/gz/.local/state/fpv-test/simitl-pr0p/`
  - `logs/simitl_pr0p/`
- Rollback must be one-directory deletion plus any explicitly listed desktop
  launcher/runtime cache cleanup.

## Non-goals

- Do not build a new simulator product.
- Do not replace `docs/sitl-*` or Gazebo acceptance gates.
- Do not use multiplayer or online competitive modes.
- Do not tune the final real drone here.
- Do not link GPL SimITL code into the main Kenet package during this spike.

## Safe Probe Suite

The safe suite summarizes P0-P6 without sending real OS input:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_probe_suite.py \
  --run-id pr0p-suite
```

For a one-page combined view of the independent path, read the latest
game-screen decision and pr0p suite evidence:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/independent_sim_readiness.py \
  --run-id independent-sim
```

This command is read-only: it does not launch pr0p, Gazebo, Betaflight,
capture, or `uinput`. It treats evidence older than 24 hours as stale by
default, so stale game-screen evidence must be refreshed before pr0p evidence
can be promoted. It also schema-checks the game-screen decision against the
current required phase set; a `SIMPLE_SANDBOX_READY` file that does not declare
and pass `S13-binding-dry` or any later required phase is treated as
`MISMATCH`. Its next-action ordering is intentionally conservative: if
P1 install-discovery is still `WAITING`, it points to the isolated
install/download step before suggesting a live pr0p race. If
`P1-client-executable` is still `WAITING`, it points to the manual
updater/install step, `pr0p_updater_runner.py`, and `pr0p_client_probe.py`
before suggesting a live race.

For the actual objective, use `pr0p_core_goal_report.py` and read
`completion_status`, not only `status`. `status=PASS` means RC/manual flight,
camera/tracker, and autopilot-control evidence are present. The full isolated
sim objective is only done when `completion_status=COMPLETE`; until then
`completion_missing` lists the remaining gates such as `moving_target_yaw`,
`approach_pitch`, or `handoff`.
`independent_sim_readiness.py` mirrors this as `objective_completion_status`
and `objective_completion_missing`, so `INDEPENDENT_SIM_READY` can be reported
without implying that the autonomous-follow objective is finished.

Expected before pr0p is running: overall `WAITING`, with P2/P3/P5 identifying
which live pr0p surfaces are not visible/reachable yet, and P6 waiting for a
selected tracking bbox. After selecting a target, pass:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_probe_suite.py \
  --tracking-bbox x,y,w,h \
  --run-id pr0p-suite-with-bbox
```

The safe suite now includes read-only `P2-fc-status-readonly` and
`P5-rc-baseline` steps. They do not send OS input or MSP RC writes; they only
prove whether FC arm/mode status and `MSP_RC` can be read, and whether the RC
baseline is stable enough for later response tests.

## Phase P0 - Preflight

Purpose: decide whether the environment can run the probe.

Isolation gate:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_isolation_check.py \
  --run-id pr0p-isolation
```

This fails if the isolated pr0p/game-screen runtime code imports or commands the
Gazebo launcher/debug path.

Checklist:

- Confirm GPU/display path:
  - `echo "$XDG_SESSION_TYPE"`
  - `echo "$DISPLAY"`
  - `glxinfo -B` if available
- Confirm Python env still passes:
  - `fpv_env/bin/python -m pytest -q`
- Confirm input path options:
  - physical joystick optional
  - virtual input path preferred for automation
- Confirm capture path options:
  - X11: `mss` / OpenCV / ffmpeg x11grab
  - Wayland: PipeWire / portal capture
  - fallback: OBS/virtual camera only if needed
- Decide install root:
  - recommended: `/tmp/fpv-test-simitl-pr0p/` for first run

Exit:

- Written preflight note with OS/session, display, GPU, and chosen capture path.

Current tool:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/preflight_probe.py \
  --install-root /tmp/fpv-test-simitl-pr0p \
  --run-id pr0p-preflight
```

Expected before pr0p is running: `msp_websocket_port` is `WAITING`, while
display/capture, input, and install-root checks should explain whether this
machine is ready for the manual pr0p smoke.

## Phase P1 - pr0p Manual Smoke

Purpose: prove pr0p itself can run and expose a virtual FC.

Steps:

1. Run the install discovery probe against the official download page.
2. If discovery returns `WAITING`, run the same probe with `--download` to put
   the updater in the isolated install root. Do not execute the updater from an
   automated gate.
3. Manually run/install pr0p from the isolated root, then rerun discovery until
   the report can see an installed updater/client candidate.
4. Start pr0p.
5. Start a local race. The virtual FC is expected to exist only during a race.
6. Connect Betaflight Configurator / web configurator manually:

   ```text
   ws://127.0.0.1:5761
   ```

7. Record:
   - pr0p version
   - map/quad used
   - whether Configurator connects
   - API/version info if visible
   - whether rates/PID page is readable
   - whether arm/disarm works manually

Acceptance:

- pr0p local race runs for at least 60 seconds.
- Configurator connects to `ws://127.0.0.1:5761`.
- Manual arm/disarm and basic stick input work.
- Exit leaves no stuck pr0p/SimITL processes.

Current install/download probe:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_install_probe.py \
  --install-root /tmp/fpv-test-simitl-pr0p \
  --run-id pr0p-install
```

To download the updater into the isolated install root without executing it:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_install_probe.py \
  --install-root /tmp/fpv-test-simitl-pr0p \
  --download \
  --run-id pr0p-install-download
```

After manually running the updater, rerun the discovery command. The report scans
the isolated root and lists executable client candidates. Then run the dedicated
client executable gate:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_updater_runner.py \
  --install-root /tmp/fpv-test-simitl-pr0p \
  --run-id pr0p-updater-dry
```

If you want Codex to launch the updater window from the isolated root, the
guarded command requires explicit acknowledgement flags:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_updater_runner.py \
  --install-root /tmp/fpv-test-simitl-pr0p \
  --launch-updater \
  --ack-external-binary \
  --leave-running \
  --ack-leave-running \
  --run-id pr0p-updater-launch
```

After the updater/manual installer finishes:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_client_probe.py \
  --install-root /tmp/fpv-test-simitl-pr0p \
  --run-id pr0p-client
```

Known menu path helper:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_ui_smoke.py \
  --run-id pr0p-ui-dry
```

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_ui_smoke.py \
  --send-ui \
  --ack-live-ui \
  --run-id pr0p-ui-live
```

This XTEST helper only clicks through `skip -> Local -> Time attack -> quad ->
scene -> track`. It is not an acceptance gate by itself; P2 websocket and
P2-msp-readonly must still prove that the virtual FC opened.

Bounded live-session runner:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_live_session_runner.py \
  --run-id pr0p-live-session-dry
```

The default runner invocation does not launch pr0p. It verifies that the
isolated executable exists and reports the current pr0p
`Controls -> RC Channels` mapping as `PASS` or `WAITING`.

For a bounded live run:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_live_session_runner.py \
  --launch-pr0p \
  --ack-live-launch \
  --send-ui \
  --ack-live-ui \
  --run-suite \
  --run-id pr0p-live-session
```

The runner starts only the pr0p process from the isolated root, can click the
known local-race path, runs the safe suite, and then cleans up the process it
started. If RC channels are still mapped to the physical transmitter, the run
must remain `WAITING` for live control until the pr0p `Controls -> RC Channels`
screen or the backup-backed config patch maps the virtual axes.

RC-source contention check:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/msp_ws_rc_probe.py \
  --run-id pr0p-rc-baseline
```

The baseline must be stable before any response gate is trusted. If a pilot is
moving a physical controller, this should report `RC_INPUT_CONTENTION` instead
of silently letting a live command test produce misleading results. Optional
low-throttle, arm-low write/readback loopback:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/msp_ws_rc_probe.py \
  --write \
  --ack-live-msp-write \
  --run-id pr0p-rc-loopback
```

Current live finding: pr0p/BF can acknowledge `MSP_SET_RAW_RC`, but `MSP_RC`
may stay at the baseline values. Treat that as a WAITING condition for RC source
priority / MSP override mode, not as proof that visual command response works.

If pr0p is not reacting to Kenet virtual RC output, check the local
`Controls -> RC Channels` mapping before response tests. The safe config probe:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_input_config_probe.py \
  --run-id pr0p-input-config
```

When it reports `PR0P_VIRTUAL_INPUT_MAPPING_MISSING`, use the mapping assistant
while the pr0p Controls page is open:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_input_config_patch.py \
  --run-id pr0p-input-config-patch-dry
```

If the dry-run patch verifies, it can be applied with an automatic backup:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_input_config_patch.py \
  --write \
  --ack-config-write \
  --run-id pr0p-input-config-patch-write
```

Before or after applying the patch, the restore path can be dry-run:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_input_config_restore.py \
  --run-id pr0p-input-config-restore-dry
```

To restore the latest Codex backup, snapshotting the current config first:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_input_config_restore.py \
  --restore \
  --ack-config-restore \
  --run-id pr0p-input-config-restore-write
```

If direct config patching is not desired, use the mapping assistant while the
pr0p Controls page is open. `--role all` now pulses roll, pitch, throttle, yaw,
and AUX1/ARM CH5; use `--role aux1` for the current AUX-only blocker:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_rc_channel_mapping_assistant.py \
  --uinput \
  --ack-live-input \
  --role all \
  --run-id pr0p-rc-channel-map-live
```

Failure labels:

- `PR0P_NO_LAUNCH`
- `PR0P_NO_RACE`
- `MSP_WS_NO_CONNECT`
- `INPUT_MAPPING_FAIL`
- `PROCESS_CLEANUP_FAIL`

## Phase P2 - MSP / FC Probe From Kenet Tools

Purpose: prove our existing MSP tooling can see the virtual FC.

Steps:

1. Start pr0p local race.
2. Try direct Kenet MSP smoke against the websocket endpoint only if supported
   by our transport. If not supported, document the gap.
3. If pr0p exposes TCP MSP directly in addition to websocket, try:

   ```bash
   fpv_env/bin/python tools/kenet_msp_smoke.py --msp-tcp 127.0.0.1:5761
   ```

4. If only websocket exists, add a small isolated bridge/probe later:

   ```text
   experiments/simitl_pr0p_probe/msp_ws_probe.py
   ```

Current tool:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/msp_ws_probe.py \
  --host 127.0.0.1 \
  --port 5761 \
  --run-id pr0p-ws
```

This verifies only the websocket handshake used by Configurator. The next
read-only semantic probe verifies that the endpoint carries MSP bytes:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/msp_ws_semantic_probe.py \
  --host 127.0.0.1 \
  --port 5761 \
  --run-id pr0p-msp-ws
```

This sends only `MSP_API_VERSION`. It does not arm, throttle, or write RC
channels.

Then read FC arm/mode status without sending commands:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/msp_ws_status_probe.py \
  --host 127.0.0.1 \
  --port 5761 \
  --run-id pr0p-fc-status
```

This prefers `MSP_STATUS_EX` and falls back to basic `MSP_STATUS` if needed.
`FC_NOT_ARMED` and `ARMING_DISABLED:*` notes are diagnostic evidence for the
current vehicle/view response blocker.

Acceptance:

- Read API/version/status/RC from the virtual FC, or precisely document that
  only Configurator-style websocket is available.
- `P2-websocket`, `P2-msp-readonly`, and `P2-fc-status-readonly` must pass
  before response or tracking-control gates can be promoted.
- No changes to `kenet/msp.py` until the transport requirement is proven.

Failure labels:

- `MSP_TRANSPORT_UNSUPPORTED`
- `MSP_READ_FAIL`
- `MSP_SINGLE_CLIENT_CONFLICT`

## Phase P3 - Image Capture Probe

Purpose: prove Kenet can receive the exact FPV view.

Candidate paths:

1. Window capture:
   - capture the pr0p game window
   - crop to FPV viewport
   - feed frames to a small adapter compatible with Kenet camera interface
2. Virtual camera:
   - only if window capture is unreliable
3. Native pr0p output/API:
   - only if documentation or local inspection reveals one

Current tool:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_capture_probe.py \
  --window-title pr0p \
  --duration 30 \
  --fps 30 \
  --min-fps 20 \
  --run-id pr0p-capture
```

Use repeated `--window-title` values or `--region left,top,width,height` if the
pr0p window title is different. The probe excludes common editor, browser, and
terminal windows by default to avoid false positives from documentation/source
file titles. Use `--crop left,top,width,height` only after a sample frame proves
the full-window capture includes non-FPV chrome.

Acceptance:

- Captures at least 20 FPS for 30 seconds.
- Frame resolution and crop are stable.
- Frame is nonblank.
- Latency estimate is recorded if possible.
- A saved sample frame shows the FPV view.

Failure labels:

- `CAPTURE_NO_WINDOW`
- `CAPTURE_BLACK_FRAME`
- `CAPTURE_TOO_SLOW`
- `CAPTURE_CROP_UNSTABLE`
- `CAPTURE_BACKEND_UNAVAILABLE`

## Phase P4 - Virtual Input Probe

Purpose: send controlled pilot input to pr0p without touching physical RC.

Candidate paths:

1. Linux virtual joystick via `uinput`.
2. Keyboard input only for manual smoke; not enough for automated PID.
3. MSP/RC write path if pr0p virtual FC accepts it and routes it as expected.

Current tool:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_virtual_input_probe.py \
  --run-id pr0p-input-dry
```

Real `uinput` readiness can be required without sending OS input:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_virtual_input_probe.py \
  --require-uinput \
  --run-id pr0p-input-ready
```

The live virtual-controller smoke is blocked unless the command includes an
explicit acknowledgement:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_virtual_input_probe.py \
  --uinput-smoke \
  --ack-live-input \
  --yaw 0.03 \
  --hold-seconds 0.1 \
  --run-id pr0p-input-live-smoke
```

pr0p must also map the expected virtual controller in its own input config:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_input_config_probe.py \
  --run-id pr0p-input-config
```

Use `--allow-physical-mapping` only when the current acceptance run is
intentionally pilot/transmitter-driven. The current observed config maps the
four primary axes to generic `<Joystick>/...` paths that match the Kenet virtual
profile, but live response has not yet proved that the runtime is using this
source.
Manual fix location: pr0p `Controls -> RC Channels`. Bind
roll/pitch/throttle/yaw to either the `Kenet Game Sandbox` virtual device for
automated tests or to the physical transmitter for pilot-only tests. Record
which device was selected in the run report before accepting P5 response gates.
Expected Kenet virtual profile: roll `ABS_X` -> `Stick/x`, pitch `ABS_Y` ->
`Stick/y`, throttle `ABS_RY` -> `RotateY`, yaw `ABS_RX` -> `RotateX`. If pr0p
stores these as generic `<Joystick>/...` paths, the input-config probe can still
accept them as compatible.

Runtime visual gate for the Controls page:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_runtime_input_visual_probe.py \
  --uinput \
  --ack-live-input \
  --axis yaw \
  --magnitude 0.6 \
  --run-id pr0p-rc-channels-visual
```

Use this only with `Controls -> RC Channels` visible, preferably cropped around
the RC bars. It proves the running UI sees virtual RC input; it does not replace
the P5 vehicle/view response gates.

Current live finding:

- pr0p Controls is reachable from an active local race through the left
  hamburger menu, then `Controls`.
- The `Input visualization` crop `480,110,320,190` produced `PASS` for yaw:
  `logs/simitl_pr0p/20260707-103654-pr0p-rc-channels-visual-controls-yaw-v1-runtime-input-visual.md`.
- Therefore the runtime UI sees virtual RC input.
- Vehicle/view response still remains `WAITING`; yaw, pitch, +throttle and
  -throttle did not move the FPV image beyond baseline drift. Treat the next
  blocker as arm/start-state/physics-input routing, not config-file mapping.
- `P2-fc-status-readonly` now exists to separate "FC is readable but not armed"
  from "FC status cannot be read" before further live response attempts.
- Live `P2-fc-status-readonly` PASS:
  `logs/simitl_pr0p/20260707-105623-pr0p-live-session-fc-status-v1-suite-p2-fc-status-msp-ws-status.md`.
  That first sample reported `FC_NOT_ARMED` with `THROTTLE,BOOTGRACE,CALIB`.
- Live monitor PASS:
  `logs/simitl_pr0p/20260707-105938-pr0p-fc-status-monitor-v1-msp-ws-status.md`.
  Across 6 samples, `BOOTGRACE` and `CALIB` cleared; persistent blocker was
  `THROTTLE` and `armed` stayed false.
- Latest decision:
  `logs/simitl_pr0p/20260707-110408-pr0p-decision-fc-arm-blocked-v1-decision.md`
  reports `FC_ARM_STATE_BLOCKED`.
- Live uinput -> MSP_RC throttle effect:
  `logs/simitl_pr0p/20260707-111545-pr0p-live-uinput-rc-effect-throttle-pos-v1-live-session.md`
  reports `THROTTLE_LOW_REACHED`; `throttle=+1.0` maps MSP_RC throttle
  `1500 -> 1000`. The opposite test
  `logs/simitl_pr0p/20260707-111509-pr0p-live-uinput-rc-effect-throttle-neg-v1-live-session.md`
  failed directionally because `throttle=-1.0` maps `1500 -> 2000`.
- Live uinput -> FC status effect:
  `logs/simitl_pr0p/20260707-112145-pr0p-live-uinput-status-throttle-low-v1-live-session.md`
  reports `UINPUT_CLEARED_THROTTLE`; while `throttle=+1.0` is held, the
  `THROTTLE` arming blocker clears. `BOOTGRACE/CALIB` may still be transient
  depending on how soon after race entry the status probe runs.
- Read-only mode ranges:
  `logs/simitl_pr0p/20260707-114048-pr0p-live-suite-mode-ranges-v1-suite-p2-mode-ranges-msp-ws-mode-ranges.md`
  reports ARM on `AUX1 / CH5`, range `1700-2100`.
- Direct button route:
  `logs/simitl_pr0p/20260707-113523-pr0p-live-uinput-arm-button-v1-live-session.md`
  reports `NO_UINPUT_ARM_BUTTON_EFFECT`; south/east buttons did not activate
  ARM mode/status while throttle was low.
- AUX1 config route:
  `logs/simitl_pr0p/20260707-114819-pr0p-input-config-aux1-dry-v1-input-config-patch.md`
  dry-runs slot 4 from empty to `<Joystick>/Z` with `real_config_write=False`.
  Before applying that patch, `logs/simitl_pr0p/20260707-115217-pr0p-live-uinput-rc-aux1-before-patch-v2-live-session.md`
  reports `NO_UINPUT_MSP_RC_EFFECT`; CH5 stays at `1500`.
- Latest manifest/decision:
  `logs/simitl_pr0p/20260707-115241-pr0p-live-manifest-aux1-before-patch-v2-live-manifest.md`
  and
  `logs/simitl_pr0p/20260707-115241-pr0p-decision-aux1-before-patch-v2-decision.md`.
  Next action is backup-backed AUX1 input patch or manual AUX1 binding, then
  `P5-uinput-rc-effect-aux1-high` and `P5-uinput-aux1-arm-status`
  verification.
- AUX1 acceptance runner:
  `experiments/simitl_pr0p_probe/pr0p_aux1_acceptance_runner.py` provides a
  plan-only report by default and an ack-gated live sequence for
  `pr0p_aux1_rc_effect` followed by `pr0p_aux1_arm_status`.

Acceptance:

- Neutral input is stable.
- Arm/disarm command can be issued or mode can be configured manually.
- Small yaw/pitch/roll/throttle commands visibly move RC bars or the vehicle.
- Input stops cleanly and returns neutral.

Failure labels:

- `UINPUT_PERMISSION_FAIL`
- `INPUT_NOT_DETECTED`
- `AXIS_ORDER_UNKNOWN`
- `NO_SAFE_NEUTRAL`

## Phase P5 - Command Response Matrix

Purpose: answer the key question: does this environment respond to small
yaw/pitch commands cleanly?

Measured answer (2026-07-07): yes, but only armed and only via attitude.
The vehicle ignores stick input while disarmed, and the FPV camera uptilt
makes pad/climb views sky-dominated so pixel-shift response estimates read
about zero even during a fast rotation. The working profile is
`--response-arm-first` (throttle-low, wait out `BOOTGRACE/CALIB`, AUX1 high,
verify `armed` read-only, hold hover throttle `-0.4`) plus
`--response-measure attitude` (signed `MSP_ATTITUDE` delta). Measured signs:
yaw `+` pulse gives `+` heading delta (`+190 deg`), pitch `+` pulse gives `-`
pitch angle (`-40.6 deg`).

Steps:

1. Start local race with fixed map/quad/config.
2. Capture FPV view and, if available, FC telemetry.
3. Send small yaw/pitch commands:
   - yaw: neutral, small left, small right
   - pitch: neutral, small forward, small back
4. Measure:
   - image motion direction
   - attitude if available
   - motor/OSD/blackbox if available
   - runaway/spin/no-spin

Current tool:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_response_probe.py \
  --axis yaw \
  --magnitude 0.05 \
  --image-axis x \
  --expected-sign 0 \
  --run-id pr0p-response-yaw-calibration
```

The default run is dry-run only and must report `WAITING`; it records image
motion if present but cannot pass acceptance because no real OS input is sent.
After P3 capture and P4 input mapping are confirmed, use:

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

If pr0p was already running before the virtual controller was created, this can
give a false `NO_VISUAL_RESPONSE`. The stronger runtime gate starts pr0p only
after creating the virtual RC device, keeps that device open, enters the local
race, and measures response through the same device:

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

Current live finding: this stronger gate can launch pr0p with persistent uinput
active and still remain `WAITING` because the visual response stays below the
minimum shift threshold. Treat that as an unresolved runtime RC-source or
in-game arming/start-state problem before attempting live tracking control.

The acceptance wrapper for this phase is:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_response_acceptance_runner.py \
  --run-id pr0p-response-acceptance-plan
```

Plan-only mode checks the latest live manifest and lists yaw/pitch live
commands without launching pr0p. It requires `P5-uinput-aux1-arm-status PASS`
before live response can run. After that prerequisite is proven:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_response_acceptance_runner.py \
  --run-live-gates \
  --ack-live-launch \
  --ack-live-ui \
  --ack-live-input \
  --yaw-expected-sign 1 \
  --pitch-expected-sign -1 \
  --run-id pr0p-response-acceptance-live
```

Acceptance:

- Small yaw command moves the view in the expected direction.
- Small pitch command moves the view in the expected direction.
- No violent spin/runaway in 5 repeated 30-second runs.
- Input and capture logs align in time.

Failure labels:

- `YAW_SIGN_FAIL`
- `PITCH_SIGN_FAIL`
- `DIRECTION_SIGN_FAIL`
- `SPIN_RUNAWAY`
- `NO_TELEMETRY`
- `TIMESTAMP_MISMATCH`
- `DRY_RUN_ONLY`

## Phase P6 - Kenet Tracker/PID Loop

Purpose: close a minimal visual loop.

Measured (2026-07-07): the loop is closed live. `pr0p_tracking_probe.py
--uinput --arm-first` arms the FC, holds hover throttle plus AUX1 under every
PID command through `HoldAxesAdapter` (tracker stop returns to the hover hold
instead of disarming mid-air), and the yaw-only run on bbox `390,475,140,85`
passed with `found_ratio 1.0`, `0` loss events, and max `|yaw|` command
`0.167` while the FC was armed with no blockers.

Initial target options:

1. Static object/gate in pr0p scene:
   - good for center-hold and yaw-only loop
2. Track editor object / custom local track object:
   - useful if a movable or high-contrast target can be placed
3. External moving overlay:
   - last resort; useful for tracker/PID but not real in-scene interaction

Steps:

1. Feed capture frames to Kenet tracker.
2. Start with manual ROI/center target.
3. Use PID output to virtual input with strict limits.
4. Run yaw-only first.
5. Add pitch/approach only after yaw is stable.

Current tool:

Run the sandbox-only E2E dry-run first. This does not need pr0p, Gazebo, a
window, or real OS input:

```bash
fpv_env/bin/python experiments/game_screen_sandbox/phase_runner.py \
  --duration 2.0 \
  --hz 10 \
  --min-tracker-found-ratio 0.8 \
  --min-loop-found-ratio 0.8 \
  --log-dir logs/simitl_pr0p \
  --run-id pr0p-synthetic-e2e
```

Then verify the generated synthetic S4 log:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_tracking_log_check.py \
  --tracking-log path/to/*-s4-loop.jsonl \
  --run-id pr0p-synthetic-log-check
```

Prepare a sample frame and bbox overlay first:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_bbox_tool.py \
  --run-id pr0p-bbox
```

Then rerun the bbox tool with the selected target:

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

The default run uses `DryRunInputAdapter`, so it proves tracker/PID command
generation but does not drive pr0p.

Verify the dry-run JSONL log independently:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_tracking_log_check.py \
  --tracking-log path/to/*-p6-tracking.jsonl \
  --run-id pr0p-tracking-log-check
```

The acceptance wrapper for live tracking is:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_tracking_acceptance_runner.py \
  --bbox x,y,w,h \
  --run-id pr0p-tracking-acceptance-plan
```

Plan-only mode checks bbox presence, the latest live manifest's P6 dry-run
status, and the latest signed response acceptance report. It will not run live
tracker/PID input until yaw and pitch response acceptance are `PASS`.

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_tracking_acceptance_runner.py \
  --bbox x,y,w,h \
  --run-live-gates \
  --ack-live-input \
  --run-id pr0p-tracking-acceptance-live
```

The lower-level live probe remains available for manual debugging:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_tracking_probe.py \
  --bbox x,y,w,h \
  --uinput \
  --ack-live-input \
  --duration 5 \
  --hz 10 \
  --run-id pr0p-tracking-live
```

Acceptance:

- Tracker `found=True` ratio is recorded.
- Center error RMS/P95 is recorded.
- PID commands are bounded.
- Sandbox-only synthetic E2E dry-run passes before live pr0p tracking is trusted.
- Sandbox-only synthetic S4 JSONL log passes `pr0p_tracking_log_check.py`.
- JSONL sample count, summary frame count, found ratio, loss events, command
  bounds, and dry-run metadata pass `pr0p_tracking_log_check.py`.
- Vehicle remains controllable for 60 seconds.
- Logs are written under `logs/simitl_pr0p/`.

Failure labels:

- `TRACKER_NO_TARGET`
- `TRACKER_LOSS`
- `TRACKER_BBOX_OUTSIDE_FRAME`
- `PID_COMMAND_UNBOUNDED`
- `PID_SATURATION`
- `VISUAL_LOOP_UNSTABLE`

## Phase P7 - Live Run Manifest

Purpose: turn the live pr0p attempt into a repeatable, evidence-backed handoff
instead of a chat-only checklist.

Current tool:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_live_run_manifest.py \
  --run-id pr0p-live
```

With a selected target bbox and calibrated signs:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_live_run_manifest.py \
  --bbox x,y,w,h \
  --yaw-expected-sign 1 \
  --pitch-expected-sign 0 \
  --run-id pr0p-live
```

Acceptance:

- Manifest names the latest suite evidence file.
- Manifest lists each P0-P6 phase status and report path when available.
- Manifest identifies the next concrete command.
- Live-input commands are visible and explicitly marked as manual/ack-required.
- Independent readiness can only promote from a real `*-live-manifest.json`.
  The manifest's `suite_report` must match the latest safe suite report, so
  refreshing the suite without refreshing the manifest keeps readiness
  `WAITING`.
- Once independent readiness is `INDEPENDENT_SIM_READY`, continue with
  `pr0p_core_goal_report.py` or `pr0p_core_next_runner.py`; those commands own
  the measured extended-follow, airborne static yaw, approach/pitch, and handoff
  gates.

Failure labels:

- `NO_SUITE_EVIDENCE`
- `MISSING_LIVE_PR0P`
- `MISSING_BBOX`
- `UNSAFE_LIVE_COMMAND_ORDER`

## Phase P8 - Automation Decision

Purpose: decide whether this is worth integrating further.

Current tool:

```bash
fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_decision_report.py \
  --run-id pr0p-decision
```

The ordered acceptance chain is the preferred live promotion path:

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
  --game-log-dir logs/game_screen_sandbox \
  --run-id pr0p-acceptance-chain-live
```

It stops in order at RC/manual flight, signed response, or live tracking.
`RC/manual flight` means AUX1/ARM plus bounded yaw/pitch/roll response; it is
not just an arm check. The arm-first + attitude profile is the proven live
profile for the current FPV view. Once the short chain passes, the next core
command is `extended_follow_live`, which calls
`pr0p_tracking_acceptance_runner.py` directly with `--duration 20.0`,
`--min-found-ratio 0.95`, `--max-loss-events 0`, `--arm-first`, and
`--arm-throttle -0.4` before moving toward the first post-core yaw-only target
scenario.
This is tracked separately as the core report's
`post_core_gates.extended_follow` gate: the short 4 s tracking acceptance can
prove core control, but it does not clear the airborne static-target gate. Once
tracking and the extended follow pass, the next core command becomes
`moving_target_yaw_plan`. It first runs a safe synthetic yaw-only regression,
then waits for real pr0p airborne static-target evidence from
`pr0p_moving_target_acceptance_runner.py`. That acceptance runner is plan-only
by default and will not send input unless
`--run-live-gate --ack-live-input --ack-airborne-static-target` are all present.
Once the synthetic yaw report exists but the real report is still missing, the
core report's next key changes from `moving_target_yaw_plan` to
`airborne_static_yaw_live`; `pr0p_core_next_runner.py` treats that command as
live and blocks execution without `--ack-live-command`.
Even with that acknowledgement, core-next reruns a read-only independent
readiness precheck and refuses to start the live command unless the combined
readiness status is still `INDEPENDENT_SIM_READY`.
Use `--precheck-live-readiness` on the plan command to run that same readiness
check without executing the live command.
The core-next JSON exposes `safe_to_execute_live_with_ack` and
`safe_to_execute_live_blockers` so automation can consume the result without
parsing human-readable notes.
It also exposes `next_objective_gate` and `next_objective_gate_snapshot`, so the
current command is tied to the exact gate it is expected to advance.
Every core-next report includes the isolation precheck as `isolation_status`,
`isolation_report`, and `isolation_violation_count`. If that status is not
`PASS`, core-next returns `GAZEBO_INDEPENDENCE_FAILED` and blocks both
`--execute-next` and `--execute-operator-next`; the independent path must not
run with forbidden Gazebo runtime coupling present.
Use `--precheck-operator-readiness` when the live operator/UI state matters.
That status-only check adds `operator_precheck_status`,
`safe_to_execute_live_with_operator_precheck`, and
`safe_to_execute_live_operator_blockers` to the same JSON without sending
uinput.
It also flattens `operator_readiness_stage`,
`operator_readiness_command_gate`,
`operator_readiness_next_action`,
`operator_readiness_recommended_command`,
`operator_readiness_recommended_candidate_action`,
`operator_readiness_freshness_gate_status`,
`operator_readiness_stale_evidence`, and
`operator_readiness_missing_evidence` so the next blocker is machine-readable
from the core-next report itself. These recommendation fields do not bypass the
combined live gate; they only identify the next operator/dry command to run.
`operator_readiness_next_command_packet` is the copy-run packet for that next
operator-side command. It prefers a discovered external-window candidate
region command when one exists, otherwise it exposes the recommended status
queue command. `live_blocker_summary.operator_next_command` repeats the chosen
command with source, uinput, unmet prerequisites, and guard fields.
Flattened discovery fields such as
`operator_readiness_window_discovery_status`,
`operator_readiness_window_discovery_reason`,
`operator_readiness_window_title_candidates`, and
`operator_readiness_excluded_window_candidates` make the actual FPV-window
blocker visible from core-next itself. The compact summary repeats the same
information through `operator_window_title_candidates` and
`operator_excluded_window_candidates`.
`--execute-operator-next` runs only that guarded operator-side command. It is
blocked if the packet sends uinput, contains live markers, has unmet
prerequisites, or still contains placeholders. Candidate/region commands also
require `--ack-operator-candidate`, because the operator must confirm that the
window candidate is the intended FPV/sim view. This lets the chain refresh
dry/capture evidence without starting the core live command.
After a guarded operator command completes, core-next reruns the status-only
operator precheck and records `post_operator_readiness_summary`,
`post_operator_precheck_status`, `post_operator_readiness_stage`,
`post_operator_missing_evidence`, `post_operator_stale_evidence`, and
`post_operator_window_discovery_status`. Treat those post fields as the proof
that the dry/capture step advanced the operator chain.
When `--operator-bbox-file` is used, that bbox must match the command target
from `--bbox` or `--bbox-file`; otherwise core-next reports
`bbox_file_mismatch` and keeps the combined live gate closed.
Use `safe_to_execute_live_with_all_prechecks` and
`safe_to_execute_live_required_blockers` as the final automation gate. Those
fields merge the independent sim readiness blockers with the operator/UI
readiness blockers, so a live command is not treated as safe just because one
side of the precheck is green.
Read `live_blocker_summary` for the compact handoff view: it records the
current objective gate, objective missing/failed evidence, independent/operator
readiness state, missing/stale operator evidence, and the next safe operator
command recommendation.
When `--execute-next --ack-live-command` is used for a live command, core-next
runs this operator readiness check by default. Use
`--skip-operator-readiness-precheck` only for an explicitly audited exception.
Use `objective_requirements` and `objective_requirement_statuses` in
`pr0p_core_goal_report.py` for the direct user-facing requirement matrix:
`gazebo_independent_path`, `target_bbox_selected`, `manual_rc_flight`,
`camera_image_to_tracker`, `autopilot_pid_control`,
`extended_closed_loop_follow`, `airborne_static_yaw_centering`,
`pitch_approach_control`, and `manual_to_auto_handoff` must all be `PASS`
before the independent pr0p/game-screen path should be treated as the measured
operating flow.
It assumes a static FPV target has been selected and the vehicle moves in the
scene, then captures the yaw-only live `*-tracking-acceptance.json` used by
`moving_target_yaw_plan` through `--real-tracking-report`. Pitch/approach
remain out of scope until yaw-only airborne static-target evidence is stable.
Once the guarded acceptance report is `PASS`, the core report's next key becomes
`airborne_static_yaw_refresh`. That command is dry: it reruns
`pr0p_moving_target_yaw_plan.py --execute-synthetic --real-tracking-report ...`
with `--ack-airborne-static-target` and does not require
`--ack-live-command` in `pr0p_core_next_runner.py`.
The real refresh requires `--ack-airborne-static-target`; it checks duration,
live input, yaw-only control, image motion, center-error convergence, final
center error, found ratio, loss count, command bounds, loop-end FC armed state,
and screen-fixed lock rejection before the gate can PASS. Real moving targets
or ghosts remain a later phase after this static-target airborne proof. After
that, `approach_pitch_plan` gates pitch/approach with synthetic range dynamics
first, then a real `--enable-pitch` tracking report refreshed with
`--real-tracking-report` and `--ack-real-approach-target`. It checks bbox-width
error reduction, final width error, nonzero bounded pitch, found ratio, loss
count, and live input before approach can PASS. Once synthetic approach is PASS
but real approach evidence is missing, the core
next key becomes `approach_pitch_live`. That guarded command uses
`pr0p_tracking_acceptance_runner.py --run-live-gates --ack-live-input
--enable-pitch` with the arm-first hover profile and remains blocked in
`pr0p_core_next_runner.py` unless `--ack-live-command` is supplied. After that
pitch-enabled tracking acceptance is PASS, the core next key becomes
`approach_pitch_refresh`, a dry `pr0p_approach_pitch_plan.py
--execute-synthetic --real-tracking-report ... --ack-real-approach-target`
refresh. After approach passes, `handoff_plan` proves the intended operator flow:
manual target centering,
follow command, tracker initialization after follow, and autonomous yaw/pitch
control with no pre-handoff auto-control samples. It runs a safe synthetic
manual-to-autonomous regression first, then waits for a real pr0p handoff report
produced by `pr0p_handoff_acceptance_runner.py` and refreshed with
`--real-handoff-report` and `--ack-real-handoff`. The acceptance runner is
guarded: it is plan-only by default and will not send uinput unless
`--run-live-gate --ack-live-input --ack-real-handoff` are all present. It runs
the follow-time `pr0p_tracking_probe.py` command, records that no auto-control
samples existed before follow, then emits the JSON consumed by `handoff_plan`.
Once synthetic handoff is PASS but real handoff evidence is missing, the core
next key becomes `handoff_live`. That guarded command runs
`pr0p_handoff_acceptance_runner.py --run-live-gate --ack-live-input
--ack-real-handoff` and remains blocked in `pr0p_core_next_runner.py` unless
`--ack-live-command` is supplied. After that real handoff acceptance is PASS,
the core next key becomes `handoff_refresh`, a dry `pr0p_handoff_plan.py
--execute-synthetic --real-handoff-report ... --ack-real-handoff` refresh.
The real gate checks follow command state, tracker init timing, bounded
yaw/pitch, tracking stability, center-error reduction, and bbox-width error
reduction before the handoff can PASS. The chain writes the final manifest plus
`pr0p_decision_refresh` and `independent_sim_readiness_refresh`.
A chain `PASS` is still not the final promotion decision by itself; inspect
those generated reports. If the pr0p decision is `PROMOTE_CANDIDATE` but
independent readiness is `WAITING`, refresh or inspect the game-screen evidence
path named by `--game-log-dir`.

Promote only if:

- P1-P7 core probes pass.
- The decision report uses a `live_manifest` source, not a safe-suite-only
  source.
- Independent readiness sees both current game-screen evidence and a live
  manifest whose `suite_report` matches the latest safe suite.
- Startup can be repeated without manual fragile steps, except launching the
  GUI client if unavoidable.
- Capture/input adapters can be deleted without touching main code.
- It gives faster tracker/PID iteration than Gazebo.
- It catches at least one class of issue Gazebo currently makes hard to isolate.

Do not promote if:

- Only screen capture works but input cannot be automated.
- It requires multiplayer/server state.
- It cannot expose enough telemetry to debug failures.
- It is less stable than the current Gazebo path.
- License constraints would force main Kenet code changes.

Promotion path:

- Keep it as `experiments/simitl_pr0p_probe/`.
- Add a single optional runbook in docs, not a default launcher.
- Add only adapter interfaces that are simulator-neutral.

## Rollback Plan

Remove:

- `experiments/simitl_pr0p_probe/`
- `/tmp/fpv-test-simitl-pr0p/`
- `/home/gz/.local/state/fpv-test/simitl-pr0p/`
- `logs/simitl_pr0p/` if no longer needed

Verify:

```bash
git status --short
pgrep -af 'pr0p|SimITL|simitl' || true
```

## First Concrete Commands

These are not final automation commands; they are the first manual probe shape.

```bash
mkdir -p /tmp/fpv-test-simitl-pr0p
fpv_env/bin/python -m pytest -q
```

Then manually:

1. Discover the pr0p Linux updater:

   ```bash
   fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_install_probe.py \
     --install-root /tmp/fpv-test-simitl-pr0p \
     --run-id pr0p-install
   ```

2. If the report says the updater was discovered, download it into the isolated
   root:

   ```bash
   fpv_env/bin/python experiments/simitl_pr0p_probe/pr0p_install_probe.py \
     --install-root /tmp/fpv-test-simitl-pr0p \
     --download \
     --run-id pr0p-install-download
   ```

3. Manually run/install pr0p from `/tmp/fpv-test-simitl-pr0p`, then rerun the
   discovery command and the safe suite.
4. Start pr0p and a local race only after both `P1-install-discovery` and
   `P1-client-executable` are no longer blocking gates in
   `independent_sim_readiness.py`.
5. Open https://app.betaflight.com in Chrome/Chromium.
6. Enable manual connection and connect to:

   ```text
   ws://127.0.0.1:5761
   ```

7. Record whether the virtual FC is visible.

## Decision Table

| Result | Decision |
| --- | --- |
| FC visible + image capture + input automation all pass | Build minimal Kenet loop probe |
| FC visible but image/input weak | Use pr0p only as Betaflight behavior reference |
| Image/input pass but FC inaccessible | Treat as game-screen sandbox, not Betaflight-in-loop |
| Setup fragile or unreproducible | Stop and keep Gazebo + simple sandbox path |
