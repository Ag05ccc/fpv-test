#!/usr/bin/env bash
set -u

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
default_fpv_root="$(cd "${script_dir}/.." && pwd)"

FPV_ROOT="${FPV_ROOT:-${default_fpv_root}}"
AEROLOOP_GAZEBO="${AEROLOOP_GAZEBO:-${FPV_ROOT}/../aeroloop_gazebo}"
BETAFLIGHT_ROOT="${BETAFLIGHT_ROOT:-${FPV_ROOT}/../betaflight}"
JOY_DEV="${JOY_DEV:-/dev/input/js0}"

missing=0

resolve_path() {
  local path="$1"
  if [ -d "$path" ]; then
    (cd "$path" && pwd)
  else
    printf '%s\n' "$path"
  fi
}

print_header() {
  printf '\n== %s ==\n' "$1"
}

check_command() {
  local name="$1"
  if command -v "$name" >/dev/null 2>&1; then
    printf '[ok] %s: %s\n' "$name" "$(command -v "$name")"
  else
    printf '[missing] %s\n' "$name"
    missing=1
  fi
}

check_path() {
  local label="$1"
  local path="$2"
  if [ -e "$path" ]; then
    printf '[ok] %s: %s\n' "$label" "$(resolve_path "$path")"
  else
    printf '[missing] %s: %s\n' "$label" "$path"
    missing=1
  fi
}

check_optional_path() {
  local label="$1"
  local path="$2"
  if [ -e "$path" ]; then
    printf '[ok] %s: %s\n' "$label" "$(resolve_path "$path")"
  else
    printf '[warn] %s not found: %s\n' "$label" "$path"
  fi
}

check_port() {
  local port="$1"
  local matches
  if ! command -v ss >/dev/null 2>&1; then
    printf '[warn] port %s: ss command not found\n' "$port"
    return
  fi
  matches="$(ss -H -lntup 2>/dev/null | awk -v port=":${port}" '$0 ~ port "([[:space:]]|$)" {print}' || true)"
  if [ -n "$matches" ]; then
    printf '[busy] port %s\n' "$port"
    printf '%s\n' "$matches" | sed 's/^/       /'
  else
    printf '[free] port %s\n' "$port"
  fi
}

print_world_step_size() {
  local label="$1"
  local path="$2"
  local step
  if [ ! -f "$path" ]; then
    return
  fi
  step="$(sed -nE 's/.*<max_step_size>[[:space:]]*([0-9.]+)[[:space:]]*<\/max_step_size>.*/\1/p' "$path" | head -n 1)"
  if [ -z "$step" ]; then
    printf '[warn] %s max_step_size: unknown\n' "$label"
    return
  fi
  if awk -v value="$step" 'BEGIN { exit !(value > 0.0025) }'; then
    printf '[warn] %s max_step_size: %s (coarse for Betaflight timing)\n' "$label" "$step"
  else
    printf '[ok] %s max_step_size: %s\n' "$label" "$step"
  fi
}

print_header "System"
if [ -r /etc/os-release ]; then
  # shellcheck disable=SC1091
  . /etc/os-release
  printf 'OS: %s\n' "${PRETTY_NAME:-unknown}"
else
  printf 'OS: unknown (/etc/os-release not readable)\n'
fi

check_command gz
if command -v gz >/dev/null 2>&1; then
  printf 'gz sim version: '
  gz sim --versions 2>/dev/null || gz --versions 2>/dev/null || printf 'unknown\n'
fi

print_header "Paths"
printf 'FPV_ROOT=%s\n' "$(resolve_path "$FPV_ROOT")"
printf 'AEROLOOP_GAZEBO=%s\n' "$(resolve_path "$AEROLOOP_GAZEBO")"
printf 'BETAFLIGHT_ROOT=%s\n' "$(resolve_path "$BETAFLIGHT_ROOT")"
printf 'JOY_DEV=%s\n' "$JOY_DEV"

check_path "Aeroloop Gazebo repo" "$AEROLOOP_GAZEBO"
check_path "Betaflight plugin" "${AEROLOOP_GAZEBO}/plugins/build/libBetaflightPlugin.so"
check_path "Betaflight SITL binary" "${BETAFLIGHT_ROOT}/obj/main/betaflight_SITL.elf"
check_optional_path "Joystick device" "$JOY_DEV"
check_optional_path "Gazebo smoke world" "${AEROLOOP_GAZEBO}/worlds/test_betaflight.sdf"
check_optional_path "Gazebo Iris world" "${AEROLOOP_GAZEBO}/worlds/betaloop_iris_betaflight_demo_harmonic.sdf"
print_world_step_size "Gazebo smoke world" "${AEROLOOP_GAZEBO}/worlds/test_betaflight.sdf"
print_world_step_size "Gazebo Iris world" "${AEROLOOP_GAZEBO}/worlds/betaloop_iris_betaflight_demo_harmonic.sdf"

print_header "Ports"
for port in 9002 9003 9004 5761 6761; do
  check_port "$port"
done

print_header "Result"
if [ "$missing" -eq 0 ]; then
  printf 'SITL/Gazebo environment looks usable.\n'
else
  printf 'SITL/Gazebo environment is missing required pieces.\n'
fi

exit "$missing"
