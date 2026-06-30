# Notes from Claude — review of the SITL / Gazebo work

Date: 2026-06-23. Scope: the uncommitted desktop-test work sitting on top of commit
`fd64554` — the new `tools/` SITL fleet (RC bridge, mixer, probe, dashboard, monitors,
log helper), the Gazebo launch/check/smoke scripts, and the doc updates. I read the
core control path (`kenet/`) and the new tools directly, and fanned out parallel readers
over the rest. These are my own opinions, with code references so you can check me.

A quick orientation that matters for everything below: there are **two** control paths
in this repo now. The production path is `kenet.py -> kenet/pipeline.py ->
kenet/controller.py -> kenet/msp.py`, which talks MSP to a real (or SITL) Betaflight and
is the thing that actually flies. The new desk-test path is `tools/sitl_rc_bridge.py +
tools/kenet_sitl_mixer.py`, which reads the Tango 2 as a USB joystick, mixes in Kenet's
pitch/yaw, and sends RC over UDP 9004 to Betaflight SITL. The mixer **does not use MSP**.
So nothing in `tools/` commands real hardware — the risks here are about test *fidelity*
and maintainability, not about a drone falling out of the sky today. Where a concern would
become a real safety issue *if the pattern moved to the MSP path*, I say so explicitly.


## Overall opinion

The headline decision is right, and the headline risk is also right there next to it.

**Right decision:** the mixer reuses the real `FlightController` and `ObjectTracker`
instead of reimplementing PID and tracking ([kenet_sitl_mixer.py:103](tools/kenet_sitl_mixer.py#L103),
[:228](tools/kenet_sitl_mixer.py#L228)). That means a desk test exercises the same control
law and the same gains that would fly. This is exactly what you want from a SITL harness and
it is the single best thing about this work.

**The risk that undercuts it:** the mixer then goes on to *reimplement the state machine*
around that controller — the AUX→state mapping, the lost-target handling, and the tracker
re-init — by hand, and that reimplementation has **already drifted** from
`kenet/pipeline.py`. So you have a test harness whose behavior differs from production in
ways that matter. A harness that disagrees with the thing it is validating gives false
confidence, which is worse than no harness. Concretely:

- **AUX thresholds use different comparisons.** Production uses strict `>`
  ([pipeline.py:234-238](kenet/pipeline.py#L234-L238)); the mixer uses `>=`
  ([kenet_sitl_mixer.py:53-58](tools/kenet_sitl_mixer.py#L53-L58)). A boundary value lands
  in a different state depending on which path you run. Small, but it is two sources of truth
  for a safety-state rule.

- **Lost-target behavior is genuinely different.** Production, on a ~2s loss, drops
  `TRACKING -> AI_ARMED` and *waits for the pilot to cycle the switch back to high* before
  re-locking ([pipeline.py:323-332](kenet/pipeline.py#L323-L332)). The mixer instead resets
  and **auto-re-inits a centered box on the very next frame**
  ([kenet_sitl_mixer.py:232-234](tools/kenet_sitl_mixer.py#L232-L234) →
  [:218-220](tools/kenet_sitl_mixer.py#L218-L220)), so it silently grabs whatever is now in
  the center with no pilot intent. This is the divergence I care about most: you could tune
  and "validate" re-acquisition in SITL and have it behave completely differently on the real
  aircraft.

My strong recommendation is to collapse these two state machines into one. Either factor the
state-machine + lost-target + re-init logic into a shared helper that both `pipeline.py` and
the mixer call, or — cleaner — have the mixer instantiate the real `TrackingPipeline` with a
pluggable output "sink" (MSP for production, UDP rc_packet for SITL). The control math is
already shared; the state machine should be too. Until it is, the SITL test is validating a
near-copy, not the real thing.

The second theme is **tooling sprawl ahead of the project's stage.** There are now three
overlapping "watch the RC/switch state" UIs — `rc_monitor.py`, `state_monitor.py`, and the
1446-line `sitl_dashboard.py` — and the dashboard fully subsumes the other two. Worse,
`state_monitor.py` reimplements the joystick/RC stack instead of importing the bridge, so a
monitor whose entire job is to show you the truth can show values that differ from what the
bridge actually sends. For a project still at the desk-test stage, one good monitor beats
three half-overlapping ones. I'd consolidate onto the dashboard and delete or thin the others.


## What is genuinely good (keep doing this)

- **Reusing the production controller/tracker** in the mixer (above). Best decision in the diff.
- **The mix rule is safe by construction.** `_mix_channels` starts from a full copy of the
  pilot channels and overwrites *only* `pitch_ch` and `yaw_ch`, and only when
  `state == TRACKING and result.found` ([kenet_sitl_mixer.py:267-281](tools/kenet_sitl_mixer.py#L267-L281)).
  Throttle, roll, and every AUX channel always pass through from the pilot, so Kenet
  structurally cannot touch throttle or arming. This matches the production guarantee that the
  controller only drives pitch and yaw, and it is the right safety posture.
- **Graceful degradation on a missing tracker.** A `TrackerUnavailableError` doesn't crash the
  mixer; it sets `tracker=None`, falls back to pilot passthrough, and labels the source
  `pilot-tracker-unavailable` ([kenet_sitl_mixer.py:247-253](tools/kenet_sitl_mixer.py#L247-L253),
  [:276-277](tools/kenet_sitl_mixer.py#L276-L277)). The `last_source` labels generally
  (`pilot` / `kenet` / `pilot-target-lost` / `pilot-tracker-unavailable`) are a nice, cheap
  debugging aid.
- **The wire format is correct and the channel order is consistent.** `pack_rc_packet` uses
  `<d16H` (timestamp + 16 channels = 40 bytes), which is exactly what Betaflight SITL expects,
  and the AETR ordering (roll 0 / pitch 1 / throttle 2 / yaw 3) matches `PipelineConfig`
  ([pipeline.py:60-63](kenet/pipeline.py#L60-L63)). RC values are clamped to 1000–2000.
- **The Gazebo UDP wiring is right.** 9002 (motor PWM out of Betaflight, into the plugin),
  9003 (sim state back), 9004 (RC in) all line up with both the plugin and SITL, and the
  smoke test starts Gazebo before SITL, which is the correct order for the UDP topology.
- **`run_gazebo_betaflight.sh` is the strongest script in the set** — `set -euo pipefail`,
  fail-fast checks with actionable hints, flexible `--world` resolution, a real `--dry-run`,
  and no hardcoded home paths.
- **The existing test suite is small and well-targeted** at the production MSP path — MSP
  framing/checksum/stale/error frames, the controller's only-pitch-and-yaw invariant, the GCS
  stop command, attitude telemetry. Good bones to extend.


## Logging and observability (the latest additions)

This part of the work I under-covered in the first pass, so here it is properly. The logging
stack is three pieces: `tools/sitl_log.py` (the JSONL writer), the mixer's per-sample flight
log ([kenet_sitl_mixer.py:283-351](tools/kenet_sitl_mixer.py#L283-L351)), and
`tools/analyze_sitl_log.py` (the offline summarizer). My overall opinion: this is the
**best-engineered new feature in the diff** — better than the monitors. It is genuinely useful
and mostly well done, with a few rough edges.

What I like:

- **The JSONL design is the right call and `sitl_log.py` is the one cleanly-shared module.**
  Line-delimited, `flush()` per record so a crash keeps everything up to the last line, sorted
  keys, and three timestamps on every record (epoch, ISO-with-offset, and `monotonic`)
  ([sitl_log.py:41-51](tools/sitl_log.py#L41-L51)). `session_start`/`session_end` bookends and a
  centralized `resolve_log_dir` (env var → default `logs/sitl`). The mixer, dashboard, and
  analyzer all import it — this is the reuse I wanted to see and didn't get from the monitors.
- **The mixer log captures exactly the right things to debug the mix and tune PID.** Each sample
  has the state, the `source` label, target found/bbox/center, full pilot vs final channels with
  per-channel deltas, and the controller's internal `yaw_error`/`forward_error`/outputs
  ([kenet_sitl_mixer.py:326-351](tools/kenet_sitl_mixer.py#L326-L351)). Crucially, the PID gains
  go into `session_start` metadata, so a log file is **self-describing** — you can tell which
  gains produced which behavior. For a tuning workflow that is the correct instinct.
- **The analyzer turns logs into findings, not just dumps.** It counts states/sources/arming
  flags, tracks max attitude and max motor spread, and ends with real heuristics — warns at
  >60° (flip/tumble) and >35° (near loss-of-control) attitude, >400µs motor spread, and detects
  "MSP offline for every sample → close Configurator" ([analyze_sitl_log.py:241-259](tools/analyze_sitl_log.py#L241-L259)).
  That is the right altitude for "did this run go badly and why."
- **`logs/` is correctly gitignored** ([.gitignore:12](.gitignore#L12), verified with
  `git check-ignore`), so unlike `eeprom.bin` the logs don't risk landing in the repo.

What I'd change:

- **Flight logging is on by default, 30 Hz, flush-every-line, with no rotation or size cap.**
  ([kenet_sitl_mixer.py:451](tools/kenet_sitl_mixer.py#L451) default `--flight-log-hz 30`;
  flush at [sitl_log.py:51](tools/sitl_log.py#L51)). Each record is large (two 16-channel arrays
  plus the controller block). On a PC for a desk test this is nothing, but this code is meant to
  end up on a Pi/Jetson, where 30 fsync-style flushes/sec of fat records to an SD card, growing
  unbounded, is the wrong default. I'd drop the default rate (10 Hz is plenty for tuning),
  buffer the flush (every N records or ~1s), and add a size/rotation cap for long sessions.
- **The analyzer can give a false all-clear on a mixer-only log.** The flip / motor-spread
  heuristics read attitude and motors from `dashboard_sample` records, which only exist when you
  ran the dashboard (it polls MSP). If you analyze a mixer-only log, `max_roll` is `None` → 0, and
  it prints "attitude stayed below 35 deg" ([analyze_sitl_log.py:242-251](tools/analyze_sitl_log.py#L242-L251)).
  A casual reader sees "below 35°" as "fine," when really there was *no attitude data at all*.
  Guard it: if there were zero dashboard/MSP samples, say "no attitude data available" instead of
  a below-threshold all-clear.
- **Channel semantics are duplicated yet again in the analyzer.** `CHANNEL_LABELS` and the
  "throttle is index 2, neutral 1000; everything else neutral 1500" rule are hardcoded
  ([analyze_sitl_log.py:16-25](tools/analyze_sitl_log.py#L16-L25),[:154-158](tools/analyze_sitl_log.py#L154-L158)).
  This is the same AETR/channel knowledge that already lives in the bridge, the mixer, and the
  monitors — now in a fifth place, with its own throttle-neutral constant. It reinforces my main
  point under "consolidate": there should be exactly one definition of the channel map/labels/
  neutrals that every tool, including the analyzer, imports.
- **The record schema is implicit and lives in three places.** The dashboard and mixer write two
  different sample shapes, and the analyzer hand-parses both. If a producer renames a field the
  analyzer silently ignores it (and the analyzer only reads `first8`, so anything past CH8 is
  invisible). A short documented schema or a shared record-builder would keep producer and
  consumer honest.

Net: keep this feature and lean on it — it's the most valuable observability you have. The fixes
are small. The one I'd actually do before trusting it for tuning is the false-all-clear guard,
because that one can quietly mislead you about whether a run was safe.


## What I would fix, in priority order

### 1. Unify the state machine (high)
Covered above. This is the most valuable single change because it restores the meaning of the
SITL test. Shared helper or pluggable-sink pipeline — either works; the current hand-rolled
copy in the mixer is the problem.

### 2. Reconcile the `aux_ch` default: 7 vs 5 (high)
Production defaults the Kenet-state switch to **AUX4 / index 7**
([pipeline.py:78](kenet/pipeline.py#L78), and `kenet.py` joystick mode follows it), but every
SITL doc, every channel-map table, and the mixer default all use **CH6 / AUX2 / index 5**
([kenet_sitl_mixer.py:431](tools/kenet_sitl_mixer.py#L431)). A reader who follows the plan
tables and then runs plain `python kenet.py --joystick` will read state from the wrong
channel — and on the MSP path that *is* the channel that triggers override, so this one is
safety-relevant on real hardware, not just confusing in SITL. Pick one (I'd move the
production default to 5 to match the Tango convention) and make the docs and code agree in one
authoritative place. Right now the only correct invocation is buried in a quick-command at the
bottom of `plan-sitl.md`.

### 3. Add the handful of tests that pin the safety glue (high)
The entire `tools/` path has zero tests, and the untested logic is exactly the safety glue.
These are pure or near-pure functions; the tests are cheap and high-value. Specifically:
- `test_mix_passthrough_when_not_tracking` — in IDLE/AI-ARMED, `final == pilot` element-for-element.
- `test_mix_overrides_only_pitch_yaw` — in TRACKING+found, only `pitch_ch`/`yaw_ch` change,
  everything else untouched, `source == "kenet"`. (This is the mixer-layer mirror of the
  controller test that already exists.)
- `test_mix_falls_back_on_target_lost` and `..._on_tracker_unavailable` — `final == pilot`,
  correct `last_source`. This is the failsafe branch and it is currently unguarded.
- `test_state_from_aux_thresholds` parametrized over 1299/1300/1699/1700, which also documents
  the `>=` vs `>` divergence in #1.
- `tests/test_sitl_rc_bridge.py` over `axis_to_rc` / `axis_to_three_pos_rc` / `clamp_rc` /
  `make_channels` / `pack_rc_packet` — sign, boundary, clamp, and the `<d16H` packet contract.
- Add a tiny `pytest.ini`/`pyproject` with `testpaths=tests` so these join the sub-second suite.

### 4. Consolidate the monitors (high, but mechanical)
Make `sitl_dashboard.py` the one observability tool; delete or reduce `rc_monitor.py` and
`state_monitor.py` to a thin `--text` shim if you want a no-browser mode. Critically, remove
`state_monitor.py`'s private `LinuxJoystick`/`_make_channels` and have it import the bridge —
a mapping monitor must never have its own copy of the mapping. Pull the shared switch/threshold
vocabulary (the 1300/1700 thresholds, the AETR ordering, the Kenet-state rule) into one small
module that the bridge, mixer, and dashboard all import, so there is exactly one definition of
"what RC value means TRACKING."

### 5. Smaller items worth a pass (medium/low)
- **Stop reaching into private fields.** The mixer reads `self.tracker._initialized`
  ([:219](tools/kenet_sitl_mixer.py#L219)) and writes `self.controller._cx/_cy`
  ([:255-256](tools/kenet_sitl_mixer.py#L255-L256)). Add `tracker.is_initialized` and
  `controller.set_frame_center(w, h)` so an internal refactor can't silently break the mixer.
- **No safe-exit RC frame / no TX watchdog.** On exit the mixer just closes the socket
  ([:133-149](tools/kenet_sitl_mixer.py#L133-L149)); it never sends a final throttle-low,
  centered frame, and if a loop tick stalls, RC tx stalls with it. SITL holds the last RC, so
  a hung mixer is a stuck-stick condition. Send a neutral frame on shutdown and warn if the
  send interval blows past ~2× the nominal period. (The packet already carries a timestamp at
  [sitl_rc_bridge.py:157](tools/sitl_rc_bridge.py#L157) that nothing currently uses — use it.)
- **The pilot↔Kenet handoff is a step, not a ramp.** When override engages/disengages the
  mixer swaps whole channel values, so a saturated Kenet yaw (up to ±300us) snaps back to the
  pilot stick instantly. Harmless in SITL, but it would be a sudden attitude command on
  hardware — worth ramping the source swap if this pattern ever moves to the MSP path.
- **`eeprom.bin` pollution / non-hermetic smoke test.** The motor smoke test runs SITL with
  cwd = repo root, so Betaflight writes `eeprom.bin` into the repo (one is already there). A
  persisted eeprom silently carries config between runs. Run SITL in a temp dir.
- **The motor smoke test's arming-flag decode is a fragile heuristic** that silently degrades
  to "no flags" on any mismatch, and its rotor-movement check compares a single before/after
  quaternion phase (can false-pass and false-fail). Gate PASS primarily on motor PWM magnitude
  (which it already reads) and parse the real MSP_STATUS_EX layout rather than searching for a
  magic byte.
- **`--headless` omits `--headless-rendering`.** The reference `aeroloop start_gazebo.sh` adds
  it; ours doesn't, which can leave Gazebo unusable on a display-less/CI box.
- **`sitl_dashboard.py` can start/stop/SIGKILL Gazebo, Betaflight, and the mixer over an
  unauthenticated HTTP POST**, and it accepts `--host`, so `--host 0.0.0.0` exposes that to the
  network. (I did not read all 1446 lines myself — worth confirming — but if so, gate the
  process-control endpoints to loopback or split them behind an opt-in flag so the default
  dashboard is a pure viewer.)
- **`CLAUDE.md` is stale.** It documents only joystick mode and the socat+MSP SITL route and
  never mentions the `tools/` RC-mixer/UDP-9004/Gazebo path that is the actual headline work.
  Since `CLAUDE.md` is auto-loaded as project instructions, the most-trusted doc is currently
  the least accurate. Refresh it, or have it defer to README/plan-sitl as the live source.
- **README uses bare `python ...`** throughout while `CLAUDE.md` insists on `fpv_env/bin/python` —
  and bare `python` is exactly the wrong-venv footgun that already cost time (the
  `TrackerCSRT unavailable` episode). State the activation step once before the first run
  command, or standardize on `fpv_env/bin/python`.


## What is *not* a problem (so you don't over-correct)

- The mix rule keeping throttle/roll/AUX on the pilot is correct and well done — don't touch it.
- The docs slightly over-state physics/PID verification, but the Turkish acceptance checklists
  in `plan-sitl.md` are honest that PID direction, the target-loss matrix, MSP transport, and
  real physics are all still open. The status is accurate where it counts; it's just easier to
  over-read maturity from the English README/`latest-development.md` lead paragraphs.
- The UDP/port wiring and packet format are correct — I verified the channel order and the
  readers verified the ports against the plugin and SITL source. No action needed there.


## If I were picking the next move

Do #1 and #3 together: as you unify the state machine, write the mix/failsafe tests against
the unified path. That single effort removes the drift, gives the harness real meaning, and
locks the safety glue — and it's the work most likely to pay off when you finally wire the MSP
transport into SITL (the next open item in `plan-sitl.md`). The monitor consolidation (#4) and
the doc/`aux_ch` reconciliation (#2) are independent and can be done any time; #2 is the one
with a real-hardware footgun, so I wouldn't let it sit too long.
