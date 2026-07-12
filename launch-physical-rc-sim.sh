#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$script_dir"

FPV_ROOT="${FPV_ROOT:-$script_dir}"
AEROLOOP_GAZEBO="${AEROLOOP_GAZEBO:-$FPV_ROOT/../aeroloop_gazebo}"
BETAFLIGHT_ROOT="${BETAFLIGHT_ROOT:-$FPV_ROOT/../betaflight}"
PYTHON_BIN="${FPV_PYTHON:-$FPV_ROOT/fpv_env/bin/python}"

device="${JOY_DEV:-/dev/input/js0}"
world="betaloop_iris_betaflight_demo_harmonic.sdf"
world_name="betaloop_demo"
max_step_size="0.001"
mode_pwm="1500"
duration="0"
diagnostics_seconds="180"
diagnostics_interval="0.25"
dashboard=1
dashboard_open=1
dashboard_host="127.0.0.1"
dashboard_port="8080"
headless=0
run_id=""
clean_existing=1
diagnostics=1
safe_start=1
safe_start_timeout="120"
dry_run=0

usage() {
  cat <<'EOF'
Usage: ./launch-physical-rc-sim.sh [options]

Tek komutla fiziksel RC + Gazebo + Betaflight SITL baslatir.
Varsayilan: Gazebo GUI acilir, Betaflight SITL calisir, /dev/input/js0 RC
kumanda UDP :9004'e gonderilir, CH7/AUX3 ANGLE icin 1500'e zorlanir.

Options:
  --device PATH             Joystick device. Default: /dev/input/js0
  --run-id NAME             Log prefix. Default: timestamped physical-rc-launch
  --duration SECONDS        0 = Ctrl-C'ye kadar calis. Default: 0
  --headless                Gazebo GUI yerine server/headless calistir.
  --world NAME              Gazebo world. Default: betaloop Iris world
  --world-name NAME         Gazebo diagnostics world name. Default: betaloop_demo
  --max-step-size VALUE     Temporary world max_step_size. Default: 0.001
  --mode-pwm PWM            Force CH7/AUX3 mode PWM. Default: 1500 (ANGLE)
  --diagnostics-seconds N   Background diagnostics capture duration. Default: 180
  --no-diagnostics          Only launch sim + RC bridge, skip diagnostic captures.
  --no-dashboard            Do not start the web dashboard.
  --dashboard-port PORT     Web dashboard port. Default: 8080
  --no-open-dashboard       Start dashboard but do not ask browser to open it.
  --no-safe-start-check     Do not wait for throttle-low / ARM-low before send.
  --safe-start-timeout N    Seconds to wait for throttle-low / ARM-low. Default: 120
  --no-clean-existing       Do not stop stale Gazebo/Betaflight/RC processes first.
  --dry-run                 Print resolved commands and exit.
  -h, --help                Show this help.

Stop:
  Press Ctrl-C in this launcher terminal. It will stop Gazebo, Betaflight and
  the RC bridge processes it started.
EOF
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --device)
      device="$2"
      shift 2
      ;;
    --run-id)
      run_id="$2"
      shift 2
      ;;
    --duration)
      duration="$2"
      shift 2
      ;;
    --headless)
      headless=1
      shift
      ;;
    --world)
      world="$2"
      shift 2
      ;;
    --world-name)
      world_name="$2"
      shift 2
      ;;
    --max-step-size)
      max_step_size="$2"
      shift 2
      ;;
    --mode-pwm)
      mode_pwm="$2"
      shift 2
      ;;
    --diagnostics-seconds)
      diagnostics_seconds="$2"
      shift 2
      ;;
    --no-diagnostics)
      diagnostics=0
      shift
      ;;
    --no-dashboard)
      dashboard=0
      shift
      ;;
    --dashboard-port)
      dashboard_port="$2"
      shift 2
      ;;
    --no-open-dashboard)
      dashboard_open=0
      shift
      ;;
    --no-safe-start-check)
      safe_start=0
      shift
      ;;
    --safe-start-timeout)
      safe_start_timeout="$2"
      shift 2
      ;;
    --no-clean-existing)
      clean_existing=0
      shift
      ;;
    --dry-run)
      dry_run=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      printf 'error: unknown option: %s\n\n' "$1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [ -z "$run_id" ]; then
  run_id="$(date +%Y%m%d-%H%M%S)-physical-rc-launch"
fi

log_dir="$FPV_ROOT/logs/sitl"
mkdir -p "$log_dir"

run_file="$log_dir/${run_id}-run-id.txt"
gazebo_log="$log_dir/${run_id}-gazebo.txt"
betaflight_log="$log_dir/${run_id}-betaflight.txt"
config_log="$log_dir/${run_id}-config.txt"
rc_log="$log_dir/${run_id}-rc-bridge.txt"
diagnostics_log="$log_dir/${run_id}-diagnostics.jsonl"
diagnostics_text_log="$log_dir/${run_id}-diagnostics.txt"
motor_udp_log="$log_dir/${run_id}-motor-udp.jsonl"
motor_udp_text_log="$log_dir/${run_id}-motor-udp.txt"
dashboard_log="$log_dir/${run_id}-dashboard.txt"
dashboard_flight_log="$log_dir/${run_id}-dashboard.jsonl"
betaflight_cwd="$log_dir/${run_id}-betaflight-cwd"

gazebo_pid=""
betaflight_pid=""
rc_pid=""
diagnostics_pid=""
motor_udp_pid=""
dashboard_pid=""
cleanup_done=0

die() {
  printf 'ERROR: %s\n' "$*" >&2
  exit 1
}

require_path() {
  local label="$1"
  local path="$2"
  [ -e "$path" ] || die "$label not found: $path"
}

stop_pid() {
  local pid="$1"
  local label="$2"
  if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
    printf 'stopping %s pid=%s\n' "$label" "$pid"
    kill "$pid" 2>/dev/null || true
    sleep 0.3
    kill -9 "$pid" 2>/dev/null || true
  fi
}

cleanup() {
  if [ "$cleanup_done" -eq 1 ]; then
    return
  fi
  cleanup_done=1
  printf '\nCleaning up launcher processes...\n'
  stop_pid "$rc_pid" "RC bridge"
  stop_pid "$dashboard_pid" "dashboard"
  stop_pid "$diagnostics_pid" "diagnostics"
  stop_pid "$motor_udp_pid" "motor UDP probe"
  stop_pid "$betaflight_pid" "Betaflight SITL"
  stop_pid "$gazebo_pid" "Gazebo launcher"
  pkill -f 'sitl_rc_bridge.py.*--send' 2>/dev/null || true
  pkill -f 'betaflight_SITL.elf' 2>/dev/null || true
  pkill -f 'gz sim.*betaloop' 2>/dev/null || true
  pkill -f '^gz sim server$' 2>/dev/null || true
  pkill -f '^gz sim gui$' 2>/dev/null || true
  printf 'RUN=%s\n' "$run_id"
  printf 'Logs: %s/%s-*\n' "$log_dir" "$run_id"
}

wait_tcp() {
  local host="$1"
  local port="$2"
  local timeout="$3"
  local deadline
  deadline=$((SECONDS + timeout))
  while [ "$SECONDS" -lt "$deadline" ]; do
    if bash -c ":</dev/tcp/${host}/${port}" >/dev/null 2>&1; then
      return 0
    fi
    sleep 0.2
  done
  return 1
}

wait_msp_ready() {
  local host="$1"
  local port="$2"
  local timeout="$3"
  "$PYTHON_BIN" - "$FPV_ROOT/tools" "$host" "$port" "$timeout" <<'PY'
import sys
import time

sys.path.insert(0, sys.argv[1])

from sitl_msp import MSP_ADVANCED_CONFIG, msp_request  # noqa: E402

host = sys.argv[2]
port = int(sys.argv[3])
deadline = time.monotonic() + float(sys.argv[4])
last_error = None

while time.monotonic() < deadline:
    try:
        msp_request(host, port, MSP_ADVANCED_CONFIG, 2.0)
        raise SystemExit(0)
    except Exception as exc:  # pragma: no cover - diagnostic helper
        last_error = exc
        time.sleep(0.2)

print("MSP not ready: %s" % last_error, file=sys.stderr)
raise SystemExit(1)
PY
}

wait_rc_safe_start() {
  "$PYTHON_BIN" - "$FPV_ROOT/tools" "$device" "$mode_pwm" "$safe_start_timeout" <<'PY'
import sys
import time

sys.path.insert(0, sys.argv[1])

from sitl_rc_bridge import CHANNEL_MAP, LinuxJoystick, apply_forced_mode_pwm, make_channels  # noqa: E402

device = sys.argv[2]
mode_pwm = int(sys.argv[3])
timeout = float(sys.argv[4])
deadline = None if timeout <= 0 else time.monotonic() + timeout
next_print = 0.0

joystick = LinuxJoystick(device)
joystick.open()
try:
    while deadline is None or time.monotonic() < deadline:
        joystick.poll(timeout=0.05)
        channels = make_channels(joystick, CHANNEL_MAP)
        apply_forced_mode_pwm(channels, mode_pwm)
        throttle = channels[2]
        arm = channels[4]
        if throttle <= 1100 and arm <= 1250:
            print("safe RC start ok: rc_us=%s" % ",".join(str(value) for value in channels[:8]))
            raise SystemExit(0)
        now = time.monotonic()
        if now >= next_print:
            print(
                "waiting for throttle low and ARM off: throttle=%d arm=%d rc_us=%s"
                % (throttle, arm, ",".join(str(value) for value in channels[:8])),
                flush=True,
            )
            next_print = now + 1.0
finally:
    joystick.close()

print("safe RC start timeout; lower throttle and turn ARM off", file=sys.stderr)
raise SystemExit(1)
PY
}

start_bg() {
  local label="$1"
  local log_path="$2"
  shift 2
  printf 'starting %s -> %s\n' "$label" "$log_path" >&2
  "$@" >"$log_path" 2>&1 &
  echo $!
}

format_cmd() {
  printf '%q ' "$@"
  printf '\n'
}

require_path "Python" "$PYTHON_BIN"
require_path "RC device" "$device"
require_path "Gazebo repo" "$AEROLOOP_GAZEBO"
require_path "Betaflight SITL binary" "$BETAFLIGHT_ROOT/obj/main/betaflight_SITL.elf"
require_path "Gazebo launcher" "$FPV_ROOT/tools/run_gazebo_betaflight.sh"

gazebo_cmd=(
  "$FPV_ROOT/tools/run_gazebo_betaflight.sh"
  --world "$world"
  --max-step-size "$max_step_size"
  --fix-iris-imu-pose
  --fix-iris-motor-map
)
if [ "$headless" -eq 1 ]; then
  gazebo_cmd+=(--headless)
fi

betaflight_cmd=("$BETAFLIGHT_ROOT/obj/main/betaflight_SITL.elf")
rc_cmd=(
  "$PYTHON_BIN" "$FPV_ROOT/tools/sitl_rc_bridge.py"
  --device "$device"
  --send
  --force-mode-pwm "$mode_pwm"
  --rate-hz 50
  --verbose
  --duration 0
)

diagnostics_samples=$("$PYTHON_BIN" - <<PY
seconds = float("$diagnostics_seconds")
interval = float("$diagnostics_interval")
print(max(1, int(round(seconds / interval))))
PY
)
diagnostics_cmd=(
  "$PYTHON_BIN" "$FPV_ROOT/tools/sitl_diagnostics.py"
  --rc-source external
  --samples "$diagnostics_samples"
  --interval "$diagnostics_interval"
  --dashboard-url "http://$dashboard_host:$dashboard_port/api/state"
  --dashboard-timeout 1.0
  --include-gazebo-pose
  --gazebo-world-name "$world_name"
  --gazebo-timeout 1.0
  --log-file "$diagnostics_log"
)
if [ "$dashboard" -eq 0 ]; then
  diagnostics_cmd+=(--direct-msp)
fi
motor_udp_cmd=(
  "$PYTHON_BIN" "$FPV_ROOT/tools/sitl_motor_udp_probe.py"
  --duration "$diagnostics_seconds"
  --log-file "$motor_udp_log"
)
dashboard_cmd=(
  "$PYTHON_BIN" "$FPV_ROOT/tools/sitl_dashboard.py"
  --device "$device"
  --host "$dashboard_host"
  --port "$dashboard_port"
  --msp-host 127.0.0.1
  --msp-port 5761
  --msp-timeout 1.5
  --msp-hz 1
  --force-mode-pwm "$mode_pwm"
  --gazebo-world "$world"
  --gazebo-max-step-size "$max_step_size"
  --log-dir "$log_dir"
  --flight-log "$dashboard_flight_log"
)
if [ "$headless" -eq 1 ]; then
  dashboard_cmd+=(--gazebo-headless)
fi
if [ "$dashboard_open" -eq 1 ]; then
  dashboard_cmd+=(--open)
fi

if [ "$dry_run" -eq 1 ]; then
  printf 'RUN=%s\n' "$run_id"
  printf 'Gazebo: '; format_cmd "${gazebo_cmd[@]}"
  printf 'Betaflight cwd: %s\n' "$betaflight_cwd"
  printf 'Betaflight: '; format_cmd "${betaflight_cmd[@]}"
  printf 'Config: mode ranges; safe manual roll/pitch/yaw rc_rate=5 rate=30 rate_limit=120; yaw/pitch PID=23,0,0\n'
  printf 'RC bridge: '; format_cmd "${rc_cmd[@]}"
  if [ "$safe_start" -eq 1 ]; then
    printf 'Safe start: wait for throttle <=1100 and ARM <=1250 for %ss\n' "$safe_start_timeout"
  else
    printf 'Safe start: disabled\n'
  fi
  if [ "$diagnostics" -eq 1 ]; then
    printf 'Diagnostics: '; format_cmd "${diagnostics_cmd[@]}"
    printf 'Motor UDP: '; format_cmd "${motor_udp_cmd[@]}"
  fi
  if [ "$dashboard" -eq 1 ]; then
    printf 'Dashboard: '; format_cmd "${dashboard_cmd[@]}"
    printf 'Dashboard URL: http://%s:%s\n' "$dashboard_host" "$dashboard_port"
  fi
  exit 0
fi

trap cleanup EXIT INT TERM

echo "$run_id" | tee "$run_file" >/dev/null
ln -sfn "$run_file" "$log_dir/latest-physical-rc-run-id.txt"
printf 'RUN=%s\n' "$run_id"
printf 'Logs: %s/%s-*\n' "$log_dir" "$run_id"

if [ "$clean_existing" -eq 1 ]; then
  printf 'Cleaning stale SITL/Gazebo/RC processes...\n'
  pkill -f 'sitl_rc_bridge.py.*--send' 2>/dev/null || true
  pkill -f "sitl_dashboard.py.*--port ${dashboard_port}" 2>/dev/null || true
  pkill -f 'betaflight_SITL.elf' 2>/dev/null || true
  pkill -f 'tools/run_gazebo_betaflight.sh' 2>/dev/null || true
  pkill -f 'gz sim.*betaloop' 2>/dev/null || true
  pkill -f '^gz sim server$' 2>/dev/null || true
  pkill -f '^gz sim gui$' 2>/dev/null || true
  sleep 1
fi

export FPV_ROOT AEROLOOP_GAZEBO BETAFLIGHT_ROOT
export PYTHONUNBUFFERED="${PYTHONUNBUFFERED:-1}"
mkdir -p "$betaflight_cwd"

gazebo_pid=$(start_bg "Gazebo" "$gazebo_log" "${gazebo_cmd[@]}")
sleep 5
if ! kill -0 "$gazebo_pid" 2>/dev/null; then
  die "Gazebo exited early; see $gazebo_log"
fi

for attempt in 1 2 3; do
  betaflight_pid=$(start_bg "Betaflight SITL attempt ${attempt}" "$betaflight_log" bash -c "cd '$betaflight_cwd' && exec '$BETAFLIGHT_ROOT/obj/main/betaflight_SITL.elf'")
  if wait_tcp 127.0.0.1 5761 10 && wait_msp_ready 127.0.0.1 5761 20; then
    break
  fi
  printf 'warning: Betaflight MSP not ready on attempt %s; restarting\n' "$attempt" >&2
  stop_pid "$betaflight_pid" "Betaflight SITL attempt ${attempt}"
  betaflight_pid=""
  sleep 2
done

if [ -z "$betaflight_pid" ]; then
  die "Betaflight MSP did not become ready after retries; see $betaflight_log"
fi

if ! (
  printf 'RUN=%s\n' "$run_id"
  printf 'Configuring mode ranges...\n'
  "$PYTHON_BIN" "$FPV_ROOT/tools/sitl_configure_modes.py" --timeout 3.0 || exit $?
  printf '\nApplying safe manual RC authority...\n'
  "$PYTHON_BIN" "$FPV_ROOT/tools/sitl_rate_config.py" \
    --timeout 3.0 \
    --roll-rc-rate 5 \
    --roll-rate 30 \
    --roll-rate-limit 120 \
    --pitch-rc-rate 5 \
    --pitch-rate 30 \
    --pitch-rate-limit 120 \
    --yaw-rc-rate 5 \
    --yaw-rate 30 \
    --yaw-rate-limit 120 || exit $?
  printf '\nApplying measured PID profile...\n'
  "$PYTHON_BIN" "$FPV_ROOT/tools/sitl_pid_config.py" \
    --timeout 3.0 \
    --yaw-p 23 --yaw-i 0 --yaw-d 0 \
    --pitch-p 23 --pitch-i 0 --pitch-d 0 || exit $?
) >"$config_log" 2>&1; then
  die "Betaflight configuration failed; see $config_log"
fi

if [ "$diagnostics" -eq 1 ]; then
  motor_udp_pid=$(start_bg "motor UDP probe" "$motor_udp_text_log" "${motor_udp_cmd[@]}")
  diagnostics_pid=$(start_bg "diagnostics" "$diagnostics_text_log" "${diagnostics_cmd[@]}")
fi

if [ "$dashboard" -eq 1 ]; then
  dashboard_pid=$(start_bg "web dashboard" "$dashboard_log" "${dashboard_cmd[@]}")
  sleep 1
  if ! kill -0 "$dashboard_pid" 2>/dev/null; then
    die "Dashboard exited early; see $dashboard_log"
  fi
fi

if [ "$safe_start" -eq 1 ]; then
  printf 'Waiting for safe RC start: throttle low, ARM off...\n'
  wait_rc_safe_start
fi

rc_pid=$(start_bg "physical RC bridge" "$rc_log" "${rc_cmd[@]}")
sleep 1
if ! kill -0 "$rc_pid" 2>/dev/null; then
  die "RC bridge exited early; see $rc_log"
fi

cat <<EOF

READY.
Gazebo, Betaflight SITL ve fiziksel RC bridge calisiyor.

Kumanda:
  - Baslangicta throttle dusuk, ARM kapali olsun.
  - CH7/AUX3 bu launcher tarafindan $mode_pwm PWM'e zorlaniyor (ANGLE).
  - ARM ac, throttle'i yavasca yukselt, buyuk yaw hareketlerinden kacin.
  - Bitirmek icin bu terminalde Ctrl-C.

RUN=$run_id
Loglar:
  $gazebo_log
  $betaflight_log
  $config_log
  $rc_log
  $dashboard_log
  $dashboard_flight_log
  $diagnostics_log
  $motor_udp_log

Dashboard:
  http://$dashboard_host:$dashboard_port

EOF

if [ "$duration" != "0" ]; then
  sleep "$duration"
else
  while true; do
    sleep 1
    if ! kill -0 "$gazebo_pid" 2>/dev/null; then
      die "Gazebo stopped; see $gazebo_log"
    fi
    if ! kill -0 "$betaflight_pid" 2>/dev/null; then
      die "Betaflight stopped; see $betaflight_log"
    fi
    if ! kill -0 "$rc_pid" 2>/dev/null; then
      die "RC bridge stopped; see $rc_log"
    fi
    if [ "$dashboard" -eq 1 ] && ! kill -0 "$dashboard_pid" 2>/dev/null; then
      die "Dashboard stopped; see $dashboard_log"
    fi
  done
fi
