#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
gazebo_root="$repo_root/gz_ws/src/ardupilot_gazebo"
overlay_root="$repo_root/simulation/ardupilot_gazebo"
gazebo_patch="$repo_root/simulation/ardupilot_gazebo.patch"

if [[ ! -f "$gazebo_root/CMakeLists.txt" || ! -f "$repo_root/ardupilot-dbox/Tools/autotest/sim_vehicle.py" ]]; then
  echo "Simulator submodules are missing. Run: git submodule update --init --recursive" >&2
  exit 1
fi

if ! git -C "$gazebo_root" apply --unidiff-zero --reverse --check "$gazebo_patch" 2>/dev/null; then
  if ! git -C "$gazebo_root" apply --unidiff-zero --check "$gazebo_patch"; then
    echo "Gazebo files differ from the pinned upstream version; review the overlay before applying it." >&2
    exit 1
  fi
  git -C "$gazebo_root" apply --unidiff-zero "$gazebo_patch"
fi

for relative_path in \
  src/DeterministicPathPlugin.cc \
  models/target_car/model.config \
  models/target_car/model.sdf \
  models/target_person/model.config \
  models/target_person/model.sdf; do
  source_file="$overlay_root/$relative_path"
  destination="$gazebo_root/$relative_path"
  if [[ -f "$destination" ]]; then
    if ! cmp -s "$source_file" "$destination"; then
      git_path="simulation/ardupilot_gazebo/$relative_path"
      known_copy=false
      for revision in $(git -C "$repo_root" log --format=%H -- "$git_path"); do
        if git -C "$repo_root" show "$revision:$git_path" | cmp -s - "$destination"; then
          known_copy=true
          break
        fi
      done
      if $known_copy; then
        cp "$source_file" "$destination"
      else
        echo "Gazebo file has local changes: $destination" >&2
        exit 1
      fi
    fi
  else
    mkdir -p "$(dirname -- "$destination")"
    cp "$source_file" "$destination"
  fi
done

echo "Gazebo simulation overlay is ready."
