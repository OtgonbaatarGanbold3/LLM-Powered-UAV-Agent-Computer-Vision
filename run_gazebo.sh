#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
gazebo_root="$repo_root/gz_ws/src/ardupilot_gazebo"
world="$gazebo_root/worlds/iris_runway.sdf"
path_plugin="$gazebo_root/build-local/libDeterministicPathPlugin.so"

"$repo_root/setup_simulation.sh"

if [[ ! -f "$path_plugin" || \
      "$gazebo_root/src/DeterministicPathPlugin.cc" -nt "$path_plugin" || \
      "$gazebo_root/CMakeLists.txt" -nt "$path_plugin" ]]; then
  echo "Building the local deterministic target-path plugin..."
  cmake -S "$gazebo_root" -B "$gazebo_root/build-local" -DCMAKE_BUILD_TYPE=RelWithDebInfo
  cmake --build "$gazebo_root/build-local" -j2
fi

export GZ_SIM_SYSTEM_PLUGIN_PATH="$gazebo_root/build-local:$gazebo_root/build${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
export GZ_SIM_RESOURCE_PATH="$gazebo_root/models:$gazebo_root/worlds${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

# Automatically enable the camera video stream 3 seconds after Gazebo starts
(
  sleep 3
  gz topic -t /world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image/enable_streaming -m gz.msgs.Boolean -p "data: 1"
) &
stream_enable_pid=$!
trap 'kill "$stream_enable_pid" 2>/dev/null || true' EXIT

# Launch Gazebo with -v1 to suppress the "Missed input frames" logging spam
gz sim -v1 -r "$world" "$@"
