#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
venv_root="${ARDUPILOT_VENV:-$HOME/venv-ardupilot}"

usage() {
    echo "Usage: $0 camera [options] | track [options] | follow [options] | gimbal [options] | world-check [options]" >&2
}

if [[ $# -eq 0 ]]; then
    usage
    exit 2
fi

action="$1"
shift
case "$action" in
    camera|track)
        script="$repo_root/ai_agent/drone_cv.py"
        dependency_check='import cv2, numpy; from gz.transport13 import Node; from gz.msgs10.image_pb2 import Image'
        ;;
    follow)
        script="$repo_root/ai_agent/follow_target.py"
        dependency_check='import cv2, numpy, pymavlink; from gz.transport13 import Node; from gz.msgs10.image_pb2 import Image'
        ;;
    gimbal)
        script="$repo_root/ai_agent/move_gimbal.py"
        dependency_check='import pymavlink'
        ;;
    world-check)
        script="$repo_root/ai_agent/check_target_motion.py"
        dependency_check='from gz.transport13 import Node; from gz.msgs10.pose_v_pb2 import Pose_V'
        ;;
    *)
        usage
        exit 2
        ;;
esac

has_dependencies() {
    python3 -c "$dependency_check" >/dev/null 2>&1
}

if ! has_dependencies; then
    if [[ ! -f "$venv_root/bin/activate" ]]; then
        echo "Python dependencies for '$action' are missing." >&2
        echo "Expected a virtual environment at: $venv_root" >&2
        echo "Set ARDUPILOT_VENV to use a different environment." >&2
        exit 1
    fi
    set +u
    source "$venv_root/bin/activate"
    set -u
fi

if ! has_dependencies; then
    echo "The environment at $venv_root does not contain the '$action' dependencies." >&2
    exit 1
fi

exec python3 "$script" "$@"
