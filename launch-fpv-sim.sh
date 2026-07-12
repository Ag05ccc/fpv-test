#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$script_dir"

FPV_ROOT="${FPV_ROOT:-$script_dir}"
AEROLOOP_GAZEBO="${AEROLOOP_GAZEBO:-$FPV_ROOT/../aeroloop_gazebo}"
BETAFLIGHT_ROOT="${BETAFLIGHT_ROOT:-$FPV_ROOT/../betaflight}"
PYTHON_BIN="${FPV_PYTHON:-$FPV_ROOT/fpv_env/bin/python}"

input="keyboard"
device="${JOY_DEV:-/dev/input/js0}"
world="betaloop_iris_betaflight_demo_populated.sdf"
max_step_size="0.0025"
mode_pwm="1500"
stick_step="100"
throttle_step="20"
run_id=""
clean_existing=1
safe_start_timeout="120"
dry_run=0

# Manual-control stability limits (measured 2026-07-04). The yaw rate-PID P is
# the one that matters: P=23 diverges into a runaway spin the instant any yaw
# command arrives (measured 50 Hz peak 7655 deg/s), P<=6 is stable (peak
# ~2.5 deg/s). So default yaw P is the STABLE value here — this is what makes
# manual a/d yaw controllable instead of spinning. Rates (rc_rate/rate/
# rate_limit) stay gentle so each stick unit is a small, bounded angular rate.
yaw_p="6"
pitch_p="23"
roll_rate="30"
pitch_rate="30"
yaw_rate="30"
rc_rate="5"
rate_limit="120"
# Yaw command clamp (us from center). Measured: this plant spins on any
# sustained yaw beyond ~+/-10 us even at the stable yaw P, so manual yaw is
# clamped small here — a held a/d key (or RC yaw) stays in the stable window
# and cannot spin the drone. Roll/pitch/throttle are unclamped and fully
# controllable. Raise this to experiment, but expect a spin (plant limit, not
# a tuning one). 0 = no clamp.
yaw_authority="10"

usage() {
  cat <<'EOF'
Usage: ./launch-fpv-sim.sh [options]

Tek komutla FPV simulasyonu baslatir: Gazebo GUI (populated world, insan +
arac modelleri), dronun ileri bakan FPV kamerasi ImageDisplay panelinde,
Betaflight SITL ve RC girisi. Varsayilan giris klavyedir; fiziksel donanim
gerekmez.

Zamanlama: kamera render yuku sync'siz dunyanin yapay-zamanlama dengesini
bozup ucusta flip'e yol acar (2026-07-03 olcumu); bu launcher bu yuzden
gercekci-zamanlama profilini kullanir: KENET_SITL_LOOPTIME_US=step ve
configs/fpv-sim.txt eeprom import'u. Bu konfig SITL arm-guvenligini
(small_angle=180 -> ANGLE arming-disable'i atlatir, feature -3D, motor PWM,
runaway off) olculmus roll P-only tune ile birlestirir, boylece dron duz
duruşta guvenle arm olur. Yamali Betaflight SITL build'i gerekir (zaten kurulu).

Klavye kontrolleri (varsayilan --input keyboard):
  w / s      throttle artir / azalt (kalici)     x  throttle'i 1000'e cek
  a / d      yaw sol / sag (anlik)               e  ARM ac / kapat
  i / k      pitch ileri / geri (anlik)          bosluk  PANIC disarm
  j / l      roll sol / sag (anlik)              1/2/3   Kenet state
  q / ESC    guvenli cikis (disarm burst gonderir)

Options:
  --input MODE            keyboard (varsayilan) veya joystick.
  --device PATH           Joystick device (yalnizca --input joystick).
                          Default: /dev/input/js0
  --world NAME            Gazebo world. Default: populated demo world
                          (betaloop_iris_betaflight_demo_populated.sdf)
  --max-step-size VALUE   Gecici world max_step_size. Default: 0.0025
  --mode-pwm PWM          CH7/AUX3 flight mode PWM. Default: 1500 (ANGLE)
  --stick-step US         Klavye anlik stick sapmasi. Default: 100
  --throttle-step US      Klavye throttle adimi. Default: 20
  --yaw-p P               Betaflight yaw rate-PID P. Default: 6 (KARARLI).
                          P=23 yaw komutunda spin attirir; 6 kontrol edilebilir.
  --pitch-p P             Betaflight pitch rate-PID P. Default: 23
  --yaw-rate R            Yaw super-rate (deg/s olceginde). Default: 30
  --rate-limit R          Roll/pitch/yaw rate_limit (max deg/s). Default: 120
  --yaw-authority US      Yaw komut kelepcesi (merkezden +/- us). Default: 10.
                          Plant her turlu surekli yaw'da ~+/-10 us ustunde spin
                          attigi icin yaw kucuk tutulur; roll/pitch/throttle tam.
                          Yukseltirsen spin bekle (plant sinirlamasi). 0 = kapali.
  --run-id NAME           Log prefix. Default: timestamped fpv-launch
  --safe-start-timeout N  Joystick modunda throttle-low/ARM-off bekleme suresi.
  --no-clean-existing     Eski Gazebo/Betaflight/RC sureclerini oldurme.
  --dry-run               Cozulen komutlari yaz ve cik.
  -h, --help              Bu yardim.

Cikis: klavye modunda q/ESC, joystick modunda Ctrl-C.
EOF
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --input)
      input="$2"
      shift 2
      ;;
    --device)
      device="$2"
      shift 2
      ;;
    --world)
      world="$2"
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
    --stick-step)
      stick_step="$2"
      shift 2
      ;;
    --throttle-step)
      throttle_step="$2"
      shift 2
      ;;
    --yaw-p)
      yaw_p="$2"
      shift 2
      ;;
    --pitch-p)
      pitch_p="$2"
      shift 2
      ;;
    --yaw-rate)
      yaw_rate="$2"
      shift 2
      ;;
    --rate-limit)
      rate_limit="$2"
      shift 2
      ;;
    --yaw-authority)
      yaw_authority="$2"
      shift 2
      ;;
    --run-id)
      run_id="$2"
      shift 2
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

case "$input" in
  keyboard|joystick) ;;
  *)
    printf 'error: --input must be keyboard or joystick, got: %s\n' "$input" >&2
    exit 2
    ;;
esac

if [ -z "$run_id" ]; then
  run_id="$(date +%Y%m%d-%H%M%S)-fpv-launch"
fi

log_dir="$FPV_ROOT/logs/sitl"
mkdir -p "$log_dir"

gazebo_log="$log_dir/${run_id}-gazebo.txt"
betaflight_log="$log_dir/${run_id}-betaflight.txt"
config_log="$log_dir/${run_id}-config.txt"
rc_log="$log_dir/${run_id}-rc-bridge.txt"
betaflight_cwd="$log_dir/${run_id}-betaflight-cwd"

gazebo_pid=""
betaflight_pid=""
rc_pid=""
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
  printf '\nCleaning up FPV launcher processes...\n'
  stop_pid "$rc_pid" "RC input"
  stop_pid "$betaflight_pid" "Betaflight SITL"
  stop_pid "$gazebo_pid" "Gazebo launcher"
  pkill -f 'sitl_keyboard_rc.py' 2>/dev/null || true
  pkill -f 'sitl_rc_bridge.py.*--send' 2>/dev/null || true
  pkill -f 'betaflight_SITL.elf' 2>/dev/null || true
  pkill -f 'g[z] sim.*betaloop' 2>/dev/null || true
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
require_path "Gazebo repo" "$AEROLOOP_GAZEBO"
require_path "Betaflight SITL binary" "$BETAFLIGHT_ROOT/obj/main/betaflight_SITL.elf"
require_path "Gazebo launcher" "$FPV_ROOT/tools/run_gazebo_betaflight.sh"
require_path "FPV GUI config" "$FPV_ROOT/tools/fpv_gui.config"
require_path "FPV SITL config" "$FPV_ROOT/configs/fpv-sim.txt"

looptime_us=$(awk -v s="$max_step_size" 'BEGIN { printf "%d", s * 1000000 }')
if [ "$looptime_us" -lt 100 ] || [ "$looptime_us" -gt 10000 ]; then
  die "--max-step-size $max_step_size is outside the looptime-sync range (100..10000 us)"
fi
if [ "$input" = "joystick" ]; then
  require_path "RC device" "$device"
fi

gazebo_cmd=(
  "$FPV_ROOT/tools/run_gazebo_betaflight.sh"
  --world "$world"
  --max-step-size "$max_step_size"
  --fix-iris-imu-pose
  --fix-iris-motor-map
  --iris-forward-camera
  --gui-config "$FPV_ROOT/tools/fpv_gui.config"
)

betaflight_cmd=("$BETAFLIGHT_ROOT/obj/main/betaflight_SITL.elf")

if [ "$input" = "keyboard" ]; then
  rc_cmd=(
    "$PYTHON_BIN" "$FPV_ROOT/tools/sitl_keyboard_rc.py"
    --mode-pwm "$mode_pwm"
    --stick-step "$stick_step"
    --throttle-step "$throttle_step"
    --yaw-authority "$yaw_authority"
    --rate-hz 50
  )
else
  rc_cmd=(
    "$PYTHON_BIN" "$FPV_ROOT/tools/sitl_rc_bridge.py"
    --device "$device"
    --send
    --force-mode-pwm "$mode_pwm"
    --yaw-authority "$yaw_authority"
    --rate-hz 50
    --verbose
    --duration 0
  )
fi

if [ "$dry_run" -eq 1 ]; then
  printf 'RUN=%s\n' "$run_id"
  printf 'Input: %s\n' "$input"
  printf 'Gazebo: '; format_cmd "${gazebo_cmd[@]}"
  printf 'Betaflight cwd: %s\n' "$betaflight_cwd"
  printf 'Betaflight env: KENET_SITL_LOOPTIME_US=%s\n' "$looptime_us"
  printf 'Betaflight eeprom import: %s\n' "$FPV_ROOT/configs/fpv-sim.txt"
  printf 'Betaflight: '; format_cmd "${betaflight_cmd[@]}"
  printf 'Config: mode ranges; rc_rate=%s roll/pitch/yaw_rate=%s/%s/%s rate_limit=%s; yaw PID P=%s (stable), pitch PID P=%s; yaw_authority=%s us\n' \
    "$rc_rate" "$roll_rate" "$pitch_rate" "$yaw_rate" "$rate_limit" "$yaw_p" "$pitch_p" "$yaw_authority"
  printf 'RC input: '; format_cmd "${rc_cmd[@]}"
  exit 0
fi

trap cleanup EXIT INT TERM

printf 'RUN=%s\n' "$run_id"
printf 'Logs: %s/%s-*\n' "$log_dir" "$run_id"

if [ "$clean_existing" -eq 1 ]; then
  printf 'Cleaning stale SITL/Gazebo/RC processes...\n'
  pkill -f 'sitl_keyboard_rc.py' 2>/dev/null || true
  pkill -f 'sitl_rc_bridge.py.*--send' 2>/dev/null || true
  pkill -f 'betaflight_SITL.elf' 2>/dev/null || true
  pkill -f 'tools/run_gazebo_betaflight.sh' 2>/dev/null || true
  pkill -f 'g[z] sim.*betaloop' 2>/dev/null || true
  sleep 1
fi

export FPV_ROOT AEROLOOP_GAZEBO BETAFLIGHT_ROOT
export PYTHONUNBUFFERED="${PYTHONUNBUFFERED:-1}"
export KENET_SITL_LOOPTIME_US="$looptime_us"
mkdir -p "$betaflight_cwd"

gazebo_pid=$(start_bg "Gazebo" "$gazebo_log" "${gazebo_cmd[@]}")
sleep 5
if ! kill -0 "$gazebo_pid" 2>/dev/null; then
  die "Gazebo exited early; see $gazebo_log"
fi

printf 'Importing sim-tune config into SITL eeprom (looptime sync %s us)...\n' "$looptime_us"
if ! (cd "$betaflight_cwd" && "$BETAFLIGHT_ROOT/obj/main/betaflight_SITL.elf" \
    --config "$FPV_ROOT/configs/fpv-sim.txt") >"$config_log" 2>&1; then
  die "Sim-tune config import failed; see $config_log"
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

# Bootstrap the Gazebo<->Betaflight lockstep handshake. Betaflight and Gazebo
# each wait for the other's first packet, so the FDM (sensor) loop never starts
# on its own: Gazebo runs free, Betaflight receives no gyro/acc, and it holds
# the ANGLE arming-disable forever (the drone will not arm even dead-level).
# Sending a short burst of zero-motor packets to Gazebo's motor port kicks the
# exchange; once running it self-sustains even while disarmed. The takeoff
# runner does the same via bootstrap_gazebo_state(); this is the launcher's
# equivalent. Measured 2026-07-04: without this, GUI runs read acc 0/0/0 and
# never arm; with it, ANGLE clears and arming works.
printf 'Bootstrapping Gazebo/Betaflight FDM loop...\n'
"$PYTHON_BIN" - "$FPV_ROOT/tools" <<'PY' || printf 'warning: FDM bootstrap send failed\n' >&2
import sys
sys.path.insert(0, sys.argv[1])
from gazebo_motor_moment_probe import send_motor_speeds
sent = send_motor_speeds([0.0, 0.0, 0.0, 0.0], 1.5, "127.0.0.1", 9002, 100.0)
print("bootstrap motor packets sent: %d" % sent)
PY

if ! (
  printf 'RUN=%s\n' "$run_id"
  printf 'Configuring mode ranges...\n'
  "$PYTHON_BIN" "$FPV_ROOT/tools/sitl_configure_modes.py" --timeout 3.0 || exit $?
  printf '\nApplying safe manual RC authority...\n'
  "$PYTHON_BIN" "$FPV_ROOT/tools/sitl_rate_config.py" \
    --timeout 3.0 \
    --roll-rc-rate "$rc_rate" \
    --roll-rate "$roll_rate" \
    --roll-rate-limit "$rate_limit" \
    --pitch-rc-rate "$rc_rate" \
    --pitch-rate "$pitch_rate" \
    --pitch-rate-limit "$rate_limit" \
    --yaw-rc-rate "$rc_rate" \
    --yaw-rate "$yaw_rate" \
    --yaw-rate-limit "$rate_limit" || exit $?
  printf '\nApplying measured PID profile (stable yaw P=%s)...\n' "$yaw_p"
  "$PYTHON_BIN" "$FPV_ROOT/tools/sitl_pid_config.py" \
    --timeout 3.0 \
    --yaw-p "$yaw_p" --yaw-i 0 --yaw-d 0 \
    --pitch-p "$pitch_p" --pitch-i 0 --pitch-d 0 || exit $?
) >>"$config_log" 2>&1; then
  die "Betaflight configuration failed; see $config_log"
fi

cat <<EOF

READY.
Gazebo GUI (FPV Camera paneli), populated world ve Betaflight SITL calisiyor.
EOF

if [ "$input" = "keyboard" ]; then
  cat <<EOF

Klavye kontrolu BU terminalde basliyor:
  w/s throttle, a/d yaw, i/k pitch, j/l roll
  e ARM, bosluk PANIC disarm, 1/2/3 Kenet state, q/ESC cikis

Ipuclari: once 'e' ile ARM, sonra 'w' ile throttle'i yavasca yukselt
(~1600-1700 hover civari). FPV goruntusu Gazebo penceresindeki
"FPV Camera" panelinde.
NOT: yaw (a/d) bilerek cok kisitli (yaw_authority=$yaw_authority us) — bu plant
gercek yaw'da spin attigi icin. Roll/pitch/throttle tam kontrol. Yaw'i acmak
istersen --yaw-authority ile yukselt (spin riski, plant sinirlamasi).

EOF
  "${rc_cmd[@]}"
  printf 'Keyboard RC exited; shutting down.\n'
else
  printf 'Waiting for safe RC start: throttle low, ARM off...\n'
  wait_rc_safe_start
  rc_pid=$(start_bg "joystick RC bridge" "$rc_log" "${rc_cmd[@]}")
  sleep 1
  if ! kill -0 "$rc_pid" 2>/dev/null; then
    die "RC bridge exited early; see $rc_log"
  fi
  cat <<EOF

Joystick RC bridge calisiyor. ARM ac, throttle'i yavasca yukselt.
Bitirmek icin bu terminalde Ctrl-C.

EOF
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
  done
fi
