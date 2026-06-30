#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
default_fpv_root="$(cd "${script_dir}/.." && pwd)"

FPV_ROOT="${FPV_ROOT:-${default_fpv_root}}"
AEROLOOP_GAZEBO="${AEROLOOP_GAZEBO:-${FPV_ROOT}/../aeroloop_gazebo}"

world_name="test_betaflight.sdf"
headless=0
headless_rendering=1
fix_iris_imu_pose=0
fix_iris_motor_map=0
iris_yaw_gyro_scale=""
iris_rotor_vel_p_gain=""
dry_run=0
verbosity=4
max_step_size=""

usage() {
  cat <<'EOF'
Usage: tools/run_gazebo_betaflight.sh [options]

Options:
  --world NAME       World file name, relative path under worlds/, or absolute path.
                     Default: test_betaflight.sdf
  --headless         Run gz sim server only, without GUI.
  --no-headless-rendering
                     Do not pass Gazebo's --headless-rendering flag with --headless.
  --max-step-size S  Run a temporary copy of the world with the first
                     <max_step_size> replaced by S, e.g. 0.001.
  --fix-iris-imu-pose
                     Use a temporary betaloop_iris_with_standoffs model copy
                     with imu_sensor pose roll changed from pi to 0.
  --fix-iris-motor-map
                     Use a temporary betaloop_iris_with_standoffs model copy
                     with BetaflightPlugin rotor ids mapped identity:
                     id0->rotor_0, id1->rotor_1, id2->rotor_2, id3->rotor_3.
  --iris-yaw-gyro-scale S
                     Use a temporary betaloop_iris_with_standoffs model copy
                     with BetaflightPlugin <yawGyroScale>S</yawGyroScale>.
  --iris-rotor-vel-p-gain G
                     Use a temporary betaloop_iris_with_standoffs model copy
                     with all rotor <vel_p_gain> values set to G.
  --dry-run          Print the command and environment without launching Gazebo.
  --verbose LEVEL    Gazebo verbosity level. Default: 4
  -h, --help         Show this help.

Environment:
  FPV_ROOT           Kenet repo root. Default: parent of this script directory.
  AEROLOOP_GAZEBO   Aeroloop Gazebo repo. Default: $FPV_ROOT/../aeroloop_gazebo
EOF
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --world)
      if [ "$#" -lt 2 ]; then
        printf 'error: --world requires a value\n' >&2
        exit 2
      fi
      world_name="$2"
      shift 2
      ;;
    --headless)
      headless=1
      shift
      ;;
    --no-headless-rendering)
      headless_rendering=0
      shift
      ;;
    --max-step-size)
      if [ "$#" -lt 2 ]; then
        printf 'error: --max-step-size requires a value\n' >&2
        exit 2
      fi
      max_step_size="$2"
      shift 2
      ;;
    --fix-iris-imu-pose)
      fix_iris_imu_pose=1
      shift
      ;;
    --fix-iris-motor-map)
      fix_iris_motor_map=1
      shift
      ;;
    --iris-yaw-gyro-scale)
      if [ "$#" -lt 2 ]; then
        printf 'error: --iris-yaw-gyro-scale requires a value\n' >&2
        exit 2
      fi
      iris_yaw_gyro_scale="$2"
      shift 2
      ;;
    --iris-rotor-vel-p-gain)
      if [ "$#" -lt 2 ]; then
        printf 'error: --iris-rotor-vel-p-gain requires a value\n' >&2
        exit 2
      fi
      iris_rotor_vel_p_gain="$2"
      shift 2
      ;;
    --dry-run)
      dry_run=1
      shift
      ;;
    --verbose)
      if [ "$#" -lt 2 ]; then
        printf 'error: --verbose requires a value\n' >&2
        exit 2
      fi
      verbosity="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      printf 'error: unknown argument: %s\n\n' "$1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if ! command -v gz >/dev/null 2>&1; then
  printf 'error: gz command not found. Install Gazebo Harmonic / gz-sim8 first.\n' >&2
  exit 1
fi

if [ ! -d "$AEROLOOP_GAZEBO" ]; then
  printf 'error: AEROLOOP_GAZEBO does not exist: %s\n' "$AEROLOOP_GAZEBO" >&2
  exit 1
fi

plugin_path="${AEROLOOP_GAZEBO}/plugins/build/libBetaflightPlugin.so"
if [ ! -f "$plugin_path" ]; then
  printf 'error: BetaflightPlugin not found: %s\n' "$plugin_path" >&2
  printf 'hint: build it in the aeroloop_gazebo repo first.\n' >&2
  exit 1
fi

case "$world_name" in
  /*)
    world_path="$world_name"
    ;;
  worlds/*)
    world_path="${AEROLOOP_GAZEBO}/${world_name}"
    ;;
  *)
    world_path="${AEROLOOP_GAZEBO}/worlds/${world_name}"
    ;;
esac

if [ ! -f "$world_path" ]; then
  if [ "${world_name##*.}" = "$world_name" ] && [ -f "${world_path}.sdf" ]; then
    world_path="${world_path}.sdf"
  else
    printf 'error: world file not found: %s\n' "$world_path" >&2
    exit 1
  fi
fi

read_world_step_size() {
  sed -nE 's/.*<max_step_size>[[:space:]]*([0-9.]+)[[:space:]]*<\/max_step_size>.*/\1/p' "$1" | head -n 1
}

world_step_size="$(read_world_step_size "$world_path" || true)"
if [ -n "$world_step_size" ]; then
  if awk -v step="$world_step_size" 'BEGIN { exit !(step > 0.0025) }'; then
    printf 'warning: world max_step_size=%s is coarse for Betaflight SITL timing.\n' "$world_step_size" >&2
    printf 'hint: retest with --max-step-size 0.001 --headless for timing isolation.\n' >&2
  fi
fi

tmp_world_dir=""
tmp_model_root=""
cleanup_tmp_world() {
  if [ -n "$tmp_world_dir" ]; then
    rm -rf "$tmp_world_dir"
  fi
  if [ -n "$tmp_model_root" ]; then
    rm -rf "$tmp_model_root"
  fi
}
trap cleanup_tmp_world EXIT

if [ -n "$max_step_size" ]; then
  if ! printf '%s\n' "$max_step_size" | grep -Eq '^[0-9]+([.][0-9]+)?$'; then
    printf 'error: --max-step-size must be numeric, got: %s\n' "$max_step_size" >&2
    exit 2
  fi
  tmp_world_dir="$(mktemp -d "${TMPDIR:-/tmp}/kenet-gazebo-world.XXXXXX")"
  tmp_world_path="${tmp_world_dir}/$(basename "$world_path")"
  if grep -q '<max_step_size>' "$world_path"; then
    sed -E "0,/<max_step_size>[[:space:]]*[^<]+[[:space:]]*<\\/max_step_size>/s//<max_step_size>${max_step_size}<\\/max_step_size>/" \
      "$world_path" > "$tmp_world_path"
  else
    cp "$world_path" "$tmp_world_path"
    printf 'warning: no <max_step_size> tag found; temporary world was not modified.\n' >&2
  fi
  world_path="$tmp_world_path"
  world_step_size="$max_step_size"
fi

if [ -n "$iris_yaw_gyro_scale" ]; then
  if ! printf '%s\n' "$iris_yaw_gyro_scale" | grep -Eq '^[0-9]+([.][0-9]+)?$'; then
    printf 'error: --iris-yaw-gyro-scale must be numeric, got: %s\n' "$iris_yaw_gyro_scale" >&2
    exit 2
  fi
fi
if [ -n "$iris_rotor_vel_p_gain" ]; then
  if ! printf '%s\n' "$iris_rotor_vel_p_gain" | grep -Eq '^[0-9]+([.][0-9]+)?$'; then
    printf 'error: --iris-rotor-vel-p-gain must be numeric, got: %s\n' "$iris_rotor_vel_p_gain" >&2
    exit 2
  fi
fi

if [ "$fix_iris_imu_pose" -eq 1 ] || [ "$fix_iris_motor_map" -eq 1 ] || [ -n "$iris_yaw_gyro_scale" ] || [ -n "$iris_rotor_vel_p_gain" ]; then
  src_model_dir="${AEROLOOP_GAZEBO}/models/betaloop_iris_with_standoffs"
  if [ ! -d "$src_model_dir" ]; then
    printf 'warning: Iris model not found, cannot apply temporary model fixes: %s\n' "$src_model_dir" >&2
  else
    tmp_model_root="$(mktemp -d "${TMPDIR:-/tmp}/kenet-gazebo-models.XXXXXX")"
    cp -a "$src_model_dir" "$tmp_model_root/"
    tmp_model_sdf="${tmp_model_root}/betaloop_iris_with_standoffs/model.sdf"
    if [ "$fix_iris_imu_pose" -eq 1 ]; then
      sed -i -E 's|<pose>0 0 0 3\.141593 0 0</pose>|<pose>0 0 0 0 0 0</pose>|' "$tmp_model_sdf"
      printf 'info: using temporary Iris model with imu_sensor pose roll=0\n' >&2
    fi
    if [ "$fix_iris_motor_map" -eq 1 ]; then
      sed -i -E '/<rotor id="0">/,/<\/rotor>/s|<jointName>[^<]+</jointName>|<jointName>rotor_0_joint</jointName>|' "$tmp_model_sdf"
      sed -i -E '/<rotor id="0">/,/<\/rotor>/s|<turningDirection>[^<]+</turningDirection>|<turningDirection>ccw</turningDirection>|' "$tmp_model_sdf"
      sed -i -E '/<rotor id="1">/,/<\/rotor>/s|<jointName>[^<]+</jointName>|<jointName>rotor_1_joint</jointName>|' "$tmp_model_sdf"
      sed -i -E '/<rotor id="1">/,/<\/rotor>/s|<turningDirection>[^<]+</turningDirection>|<turningDirection>ccw</turningDirection>|' "$tmp_model_sdf"
      sed -i -E '/<rotor id="2">/,/<\/rotor>/s|<jointName>[^<]+</jointName>|<jointName>rotor_2_joint</jointName>|' "$tmp_model_sdf"
      sed -i -E '/<rotor id="2">/,/<\/rotor>/s|<turningDirection>[^<]+</turningDirection>|<turningDirection>cw</turningDirection>|' "$tmp_model_sdf"
      sed -i -E '/<rotor id="3">/,/<\/rotor>/s|<jointName>[^<]+</jointName>|<jointName>rotor_3_joint</jointName>|' "$tmp_model_sdf"
      sed -i -E '/<rotor id="3">/,/<\/rotor>/s|<turningDirection>[^<]+</turningDirection>|<turningDirection>cw</turningDirection>|' "$tmp_model_sdf"
      printf 'info: using temporary Iris model with identity Betaflight motor map\n' >&2
    fi
    if [ -n "$iris_yaw_gyro_scale" ]; then
      if grep -q '<yawGyroScale>' "$tmp_model_sdf"; then
        sed -i -E "0,/<yawGyroScale>[[:space:]]*[^<]+[[:space:]]*<\\/yawGyroScale>/s//<yawGyroScale>${iris_yaw_gyro_scale}<\\/yawGyroScale>/" "$tmp_model_sdf"
      else
        sed -i -E "0,/<imuName>[^<]+<\\/imuName>/s//&\\n      <yawGyroScale>${iris_yaw_gyro_scale}<\\/yawGyroScale>/" "$tmp_model_sdf"
      fi
      printf 'info: using temporary Iris model with yawGyroScale=%s\n' "$iris_yaw_gyro_scale" >&2
    fi
    if [ -n "$iris_rotor_vel_p_gain" ]; then
      sed -i -E "s|<vel_p_gain>[[:space:]]*[^<]+[[:space:]]*</vel_p_gain>|<vel_p_gain>${iris_rotor_vel_p_gain}</vel_p_gain>|g" "$tmp_model_sdf"
      printf 'info: using temporary Iris model with rotor vel_p_gain=%s\n' "$iris_rotor_vel_p_gain" >&2
    fi
  fi
fi

model_path_prefix="${AEROLOOP_GAZEBO}/models"
if [ -n "$tmp_model_root" ]; then
  model_path_prefix="${tmp_model_root}:${model_path_prefix}"
fi

export SDF_PATH="${model_path_prefix}${SDF_PATH:+:${SDF_PATH}}"
export GZ_SIM_RESOURCE_PATH="${AEROLOOP_GAZEBO}/worlds:${model_path_prefix}:${AEROLOOP_GAZEBO}${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${AEROLOOP_GAZEBO}/plugins/build${GZ_SIM_SYSTEM_PLUGIN_PATH:+:${GZ_SIM_SYSTEM_PLUGIN_PATH}}"

cmd=(gz sim -r -v "$verbosity")
if [ "$headless" -eq 1 ]; then
  cmd+=( -s )
  if [ "$headless_rendering" -eq 1 ]; then
    cmd+=( --headless-rendering )
  fi
fi
cmd+=( "$world_path" )

printf 'AEROLOOP_GAZEBO=%s\n' "$AEROLOOP_GAZEBO"
printf 'SDF_PATH=%s\n' "$SDF_PATH"
printf 'GZ_SIM_RESOURCE_PATH=%s\n' "$GZ_SIM_RESOURCE_PATH"
printf 'GZ_SIM_SYSTEM_PLUGIN_PATH=%s\n' "$GZ_SIM_SYSTEM_PLUGIN_PATH"
printf 'world=%s\n' "$world_path"
printf 'world_max_step_size=%s\n' "${world_step_size:-unknown}"
printf 'fix_iris_imu_pose=%s\n' "$fix_iris_imu_pose"
printf 'fix_iris_motor_map=%s\n' "$fix_iris_motor_map"
printf 'iris_yaw_gyro_scale=%s\n' "${iris_yaw_gyro_scale:-1.0}"
printf 'iris_rotor_vel_p_gain=%s\n' "${iris_rotor_vel_p_gain:-model}"
printf 'command:'
printf ' %q' "${cmd[@]}"
printf '\n'

if [ "$dry_run" -eq 1 ]; then
  if [ -n "$tmp_world_dir" ]; then
    printf 'note: dry-run temporary world will be removed when this script exits.\n'
  fi
  exit 0
fi

if [ -n "$tmp_world_dir" ] || [ -n "$tmp_model_root" ]; then
  "${cmd[@]}"
else
  exec "${cmd[@]}"
fi
