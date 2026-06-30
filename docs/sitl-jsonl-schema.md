# SITL JSONL Log Schema

This file documents the lightweight JSONL contract shared by the Kenet SITL
tools. It is intentionally practical rather than exhaustive: logs may include
extra fields, but producers should keep the fields below stable so
`tools/analyze_sitl_log.py` and run handoffs stay useful.

## Base Record

Every record written by `JsonlLogger` has these fields:

| Field | Type | Meaning |
| --- | --- | --- |
| `event` | string | Record type. |
| `time` | number | Unix epoch seconds. |
| `time_iso` | string | Local wall-clock timestamp with timezone offset. |
| `monotonic` | number | Process monotonic timestamp for relative timing. |

`session_start` is written when a logger is opened with metadata. `session_end`
is written on close. Rotated files start with another `session_start` carrying
`rotation_index`.

## Shared RC Conventions

RC packet channels are pilot/AETR ordered for the first four inputs:
`roll`, `pitch`, `throttle`, `yaw`, then AUX channels.

Shared channel indices and AETR-to-MSP RPYT mapping live in
`kenet/rc_channels.py`. SITL labels, throttle neutral, and CH7/AUX3 forced-mode
metadata live in `tools/sitl_rc_channels.py`. New tools should import those
helpers instead of copying labels or channel neutrals.

## `kenet_mixer_sample`

Producer: `tools/kenet_sitl_mixer.py`

Required stable fields:

| Field | Type | Meaning |
| --- | --- | --- |
| `state` | string | Kenet state name: `IDLE`, `AI-ARMED`, or `TRACKING`. |
| `source` | string | RC source label such as `pilot`, `kenet`, or `pilot-target-lost`. |
| `target_found` | boolean | Whether tracker/synthetic target was found. |
| `pilot_channels` | array[int] | Full 16-channel pilot RC frame before Kenet mixing. |
| `final_channels` | array[int] | Full 16-channel frame sent to SITL after mixing. |
| `controller` | object | Controller errors, outputs, and current controller channel view. |

Optional compatibility fields:

| Field | Type | Meaning |
| --- | --- | --- |
| `first8` | object | Legacy summary with `pilot`, `final`, and `delta` for CH1-CH8. |
| `target_bbox` | array[int] or null | Tracker bbox. |
| `target_center` | array[number] or null | Target center. |
| `frame_shape` | array[int] or null | `[width, height]` when a frame exists. |
| `loop_fps` | number | Mixer loop rate estimate. |
| `send` | boolean | Whether UDP RC sending was enabled. |
| `tx_warning_count` | integer | Number of RC send interval watchdog warnings. |
| `tracker_error` | string or null | Tracker availability/error text. |

Analyzer behavior: prefer `pilot_channels` and `final_channels`. If only
`first8` exists, the analyzer uses it and prints a schema warning.

## `dashboard_sample` and `manual_marker`

Producer: `tools/sitl_dashboard.py`

Stable fields used by analysis:

| Field | Type | Meaning |
| --- | --- | --- |
| `uptime` | number | Dashboard process uptime. |
| `joystick` | object | Device, connection state, raw axes/buttons, and `pilot_channels`. |
| `kenet` | object | Kenet state snapshot from joystick/RC. |
| `autopilot_command` | object | Arm switch snapshot. |
| `autopilot_mode` | object | Flight mode switch snapshot. |
| `msp` | object | Betaflight RC, motor, attitude, active mode, and arming status. |
| `channels` | array[object] | Per-channel pilot-vs-FC rows with `label`, `pilot`, `fc`, and `delta`. |
| `links` | object | Gazebo/process/link status snapshot. |

`manual_marker` has the same shape when emitted from the dashboard, plus
`label`.

## `diagnostic_sample`

Producer: `tools/sitl_diagnostics.py`

Stable fields:

| Field | Type | Meaning |
| --- | --- | --- |
| `sample_index` | integer | 1-based sample number. |
| `sample_elapsed_ms` | number | Time spent collecting the sample. |
| `joystick` | object | RC source snapshot, virtual or external. |
| `msp` | object | Betaflight direct/dashboard MSP snapshot. |
| `gazebo` | object | Gazebo stats/link snapshot. |
| `gazebo_pose` | object | Optional pose and IMU sample. |
| `dashboard` | object | Dashboard API snapshot when used. |
| `channels` | array[object] | Pilot-vs-FC deltas using shared AETR-to-MSP mapping. |

`diagnostic_summary` contains the run-level summary produced from collected
diagnostic samples.

## Virtual RC Events

Producer: `tools/sitl_virtual_rc.py`

| Event | Stable fields |
| --- | --- |
| `virtual_rc_step_start` | `step`, `seconds`, `channels` |
| `virtual_rc_frame` | `step`, `elapsed`, `seconds`, `sent_bytes`, `channels` |
| `virtual_rc_summary` | `sent_packets` |

These logs are used by `tools/sitl_pid_sweep_summary.py` to find delayed nudge
timing.

## Motor UDP Events

Producer: `tools/sitl_motor_udp_probe.py`

| Event | Stable fields |
| --- | --- |
| `motor_udp_sample` | `ok`, `error`, `bytes`, `source`, `received_at`, plus parsed raw motor fields when `ok` is true |
| `motor_udp_summary` | `summary` object with sample count, raw spread, axis bias, and first-large-spread data |

## Producer Rules

- Keep base fields from `JsonlLogger`.
- Preserve existing stable fields when adding new data.
- Prefer full-channel arrays (`pilot_channels`, `final_channels`) over first-8
  compatibility summaries.
- Use `kenet/rc_channels.py` for production channel indices and AETR-to-MSP
  mapping; use `tools/sitl_rc_channels.py` for SITL labels and neutral values.
- If a field cannot be sampled, emit `null`, `[]`, or an object with `ok: false`
  and `error` rather than silently omitting the whole section.
