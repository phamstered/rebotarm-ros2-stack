#!/usr/bin/env bash
# Gatekeeping check before VLA / teleop data collection recording.
# Desk can SSH-run this; start_arm_teleop.sh may call with --warn-only.
#
# Hard failures (exit 1 unless --warn-only):
#   - no message from /stm32/joint_states or /stm32/status within timeout
#   - header.stamp is zero (sec==0 and nanosec==0) on either topic
#   - /stm32/status fault_code != 0
#   - /stm32/status mode != MODE_TRACK (1)
set -eo pipefail

WORKSPACE_DIR="${LEADER_FOLLOWER_WS:-$HOME/Project/rebotarm-ros2-stack}"
ROS_SETUP="/opt/ros/humble/setup.bash"
WORKSPACE_SETUP="$WORKSPACE_DIR/install/setup.bash"
DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
TOPIC_TIMEOUT="${COLLECTION_CHECK_TIMEOUT:-10}"
WARN_ONLY="false"
JOINT_STATES_TOPIC="/stm32/joint_states"
STATUS_TOPIC="/stm32/status"
# HardwareStatus.msg: MODE_HOLD=0 MODE_TRACK=1 MODE_RETURN=2 MODE_DISABLE=3
MODE_TRACK=1

usage() {
    echo "Usage: $0 [options]"
    echo
    echo "  Echo one /stm32/joint_states and /stm32/status, then gate on stamp,"
    echo "  fault_code, and mode=TRACK before starting a recording session."
    echo
    echo "  --warn-only           Print failures as WARN and exit 0 (for bring-up)."
    echo "  --timeout=N           Seconds to wait per topic (default: 10)."
    echo "  --ros-domain-id=N     ROS domain (default: current or 0)."
    echo "  --workspace=DIR       Overlay root with install/setup.bash."
}

for argument in "$@"; do
    case "$argument" in
        --warn-only)
            WARN_ONLY="true"
            ;;
        --timeout=*)
            TOPIC_TIMEOUT="${argument#*=}"
            ;;
        --ros-domain-id=*)
            DOMAIN_ID="${argument#*=}"
            ;;
        --workspace=*)
            WORKSPACE_DIR="${argument#*=}"
            WORKSPACE_SETUP="$WORKSPACE_DIR/install/setup.bash"
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "ERROR: Unknown argument: $argument" >&2
            usage >&2
            exit 2
            ;;
    esac
done

if [[ ! "$DOMAIN_ID" =~ ^[0-9]+$ ]]; then
    echo "ERROR: --ros-domain-id must be a non-negative integer." >&2
    exit 2
fi
if [[ ! "$TOPIC_TIMEOUT" =~ ^[1-9][0-9]*$ ]]; then
    echo "ERROR: --timeout must be a positive integer." >&2
    exit 2
fi
if [[ ! -f "$ROS_SETUP" ]]; then
    echo "ERROR: ROS 2 Humble setup not found: $ROS_SETUP" >&2
    exit 1
fi

source "$ROS_SETUP"
if [[ -f "$WORKSPACE_SETUP" ]]; then
    # shellcheck disable=SC1090
    source "$WORKSPACE_SETUP"
fi
export ROS_DOMAIN_ID="$DOMAIN_ID"
export RCUTILS_COLORIZED_OUTPUT=0

FAILURES=0
LAST_TOPIC_SAMPLE=""

fail() {
    if [[ "$WARN_ONLY" == "true" ]]; then
        echo "[WARN] $1" >&2
    else
        echo "[FAIL] $1" >&2
    fi
    FAILURES=$((FAILURES + 1))
}

pass() {
    echo "[PASS] $1"
}

echo "=== Collection readiness check (ROS_DOMAIN_ID=$ROS_DOMAIN_ID) ==="
echo "Topics: $JOINT_STATES_TOPIC, $STATUS_TOPIC (timeout ${TOPIC_TIMEOUT}s)"

echo_topic_once() {
    local topic="$1"
    local timeout_seconds="$2"
    local topic_output=""

    LAST_TOPIC_SAMPLE=""
    if ! topic_output="$(
        timeout "$timeout_seconds" ros2 topic echo "$topic" --once 2>&1
    )"; then
        fail "No message from $topic within ${timeout_seconds}s."
        if [[ -n "$topic_output" ]]; then
            echo "$topic_output" >&2
        fi
        return 1
    fi
    LAST_TOPIC_SAMPLE="$topic_output"
    return 0
}

# Parse std_msgs/Header stamp from ros2 topic echo YAML.
# Looks for the first stamp block:
#   stamp:
#     sec: N
#     nanosec: M
parse_header_stamp() {
    local sample="$1"
    local sec=""
    local nanosec=""

    sec="$(
        awk '
            /^[[:space:]]*stamp:[[:space:]]*$/ { in_stamp=1; next }
            in_stamp && /^[[:space:]]*sec:[[:space:]]*/ {
                print $2; exit
            }
            in_stamp && /^[^[:space:]]/ { exit }
        ' <<<"$sample"
    )"
    nanosec="$(
        awk '
            /^[[:space:]]*stamp:[[:space:]]*$/ { in_stamp=1; next }
            in_stamp && /^[[:space:]]*nanosec:[[:space:]]*/ {
                print $2; exit
            }
            in_stamp && /^[^[:space:]]/ { exit }
        ' <<<"$sample"
    )"

    if [[ ! "$sec" =~ ^-?[0-9]+$ ]] || [[ ! "$nanosec" =~ ^[0-9]+$ ]]; then
        echo ""
        return 1
    fi
    printf '%s %s\n' "$sec" "$nanosec"
}

check_stamp_non_zero() {
    local topic="$1"
    local sample="$2"
    local parsed=""
    local sec=""
    local nanosec=""

    if ! parsed="$(parse_header_stamp "$sample")"; then
        fail "$topic header.stamp could not be parsed (expected stamp.sec / stamp.nanosec)."
        echo "$sample" >&2
        return 1
    fi
    sec="${parsed%% *}"
    nanosec="${parsed#* }"
    if [[ "$sec" == "0" && "$nanosec" == "0" ]]; then
        fail "$topic header.stamp is zero (sec=0 nanosec=0)."
        echo "       Agent epoch/time is required for valid MCU stamps before recording." >&2
        echo "       See docs/data-collection.md (Agent 时间 / 非零 stamp)." >&2
        return 1
    fi
    pass "$topic header.stamp=sec:$sec nanosec:$nanosec"
}

# --- /stm32/joint_states ---
echo
echo "--- $JOINT_STATES_TOPIC ---"
if echo_topic_once "$JOINT_STATES_TOPIC" "$TOPIC_TIMEOUT"; then
    echo "$LAST_TOPIC_SAMPLE"
    check_stamp_non_zero "$JOINT_STATES_TOPIC" "$LAST_TOPIC_SAMPLE" || true
fi

# --- /stm32/status ---
echo
echo "--- $STATUS_TOPIC ---"
status_sample=""
if echo_topic_once "$STATUS_TOPIC" "$TOPIC_TIMEOUT"; then
    status_sample="$LAST_TOPIC_SAMPLE"
    echo "$status_sample"
    check_stamp_non_zero "$STATUS_TOPIC" "$status_sample" || true

    fault_code="$(awk '/^fault_code:/ {print $2; exit}' <<<"$status_sample")"
    mode="$(awk '/^mode:/ {print $2; exit}' <<<"$status_sample")"
    last_command_sequence="$(
        awk '/^last_command_sequence:/ {print $2; exit}' <<<"$status_sample"
    )"

    if [[ -n "$last_command_sequence" ]]; then
        echo "last_command_sequence=$last_command_sequence"
    else
        echo "[WARN] last_command_sequence field not present in status sample."
    fi

    if [[ ! "$fault_code" =~ ^[0-9]+$ ]]; then
        fail "status fault_code is not an integer: '$fault_code'."
    elif [[ "$fault_code" != "0" ]]; then
        if [[ "$fault_code" == "11" ]]; then
            fail "status fault_code=11 (COMMAND_LOSS_LATCHED). Do not record near HOLD/fault 11."
        else
            fail "status fault_code=$fault_code (expected 0)."
        fi
    else
        pass "status fault_code=0"
    fi

    if [[ ! "$mode" =~ ^[0-9]+$ ]]; then
        fail "status mode is not an integer: '$mode'."
    elif [[ "$mode" != "$MODE_TRACK" ]]; then
        fail "status mode=$mode (expected MODE_TRACK=$MODE_TRACK). Do not record in HOLD/RETURN/DISABLE."
    else
        pass "status mode=MODE_TRACK ($MODE_TRACK)"
    fi
fi

echo
if (( FAILURES > 0 )); then
    if [[ "$WARN_ONLY" == "true" ]]; then
        echo "[WARN] Collection readiness: $FAILURES issue(s); recording gate not enforced (--warn-only)."
        exit 0
    fi
    echo "[FAIL] Collection readiness: $FAILURES hard failure(s). Do not start recording."
    exit 1
fi

echo "[PASS] Collection readiness OK — stamps non-zero, fault_code=0, mode=TRACK."
exit 0
