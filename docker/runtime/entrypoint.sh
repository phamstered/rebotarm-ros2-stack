#!/bin/bash
set -eo pipefail
source /opt/ros/humble/setup.bash
if [[ -n "${ROBOT_EXTERNAL_SETUP:-}" ]]; then
    [[ -r "$ROBOT_EXTERNAL_SETUP" ]] || { echo "Missing external overlay: $ROBOT_EXTERNAL_SETUP" >&2; exit 2; }
    source "$ROBOT_EXTERNAL_SETUP"
fi
source /opt/rebotarm/install/setup.bash
exec "$@"
