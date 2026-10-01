#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ardupilot_root="$repo_root/ardupilot-dbox"
gimbal_params="$repo_root/gz_ws/src/ardupilot_gazebo/config/gazebo-iris-gimbal.parm"
venv_root="${ARDUPILOT_VENV:-$HOME/venv-ardupilot}"

if [[ ! -t 0 ]]; then
    echo "SITL needs an interactive terminal for MAVProxy telemetry and commands." >&2
    echo "Open a terminal and run ./run_ardupilot.sh there (or allocate a PTY)." >&2
    exit 1
fi

has_ardupilot_python() {
    python3 -c 'import pexpect, pymavlink' >/dev/null 2>&1 &&
        command -v mavproxy.py >/dev/null 2>&1
}

if ! has_ardupilot_python; then
    if [[ ! -f "$venv_root/bin/activate" ]]; then
        echo "ArduPilot Python dependencies are missing." >&2
        echo "Expected a virtual environment at: $venv_root" >&2
        echo "Set ARDUPILOT_VENV to use a different environment." >&2
        exit 1
    fi
    # The activation script may inspect optional shell variables.
    set +u
    source "$venv_root/bin/activate"
    set -u
fi

if ! has_ardupilot_python; then
    echo "The ArduPilot environment at $venv_root is missing pexpect, pymavlink, or MAVProxy." >&2
    exit 1
fi

cd "$ardupilot_root"
exec ./Tools/autotest/sim_vehicle.py \
    -v ArduCopter \
    -f gazebo-iris \
    --model JSON \
    --add-param-file="$repo_root/config/dbox.parm" \
    --add-param-file="$gimbal_params" \
    --out=127.0.0.1:14551 \
    "$@"
