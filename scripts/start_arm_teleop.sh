#!/usr/bin/env bash
# Start and verify the joint1-6 leader-to-follower ros2_control pipeline.
set -eo pipefail

WORKSPACE_DIR="${LEADER_FOLLOWER_WS:-$HOME/Project/rebotarm-ros2-stack}"
LEADER_WORKSPACE_DIR="${LEADER_ARM_WS:-$HOME/Project/Star-Arm-102/ROS2_HUMBLE}"
ROS_SETUP="/opt/ros/humble/setup.bash"
WORKSPACE_SETUP="$WORKSPACE_DIR/install/setup.bash"
LEADER_SETUP="$LEADER_WORKSPACE_DIR/install/setup.bash"
CONTROLLER_MANAGER="${CONTROLLER_MANAGER:-/controller_manager}"
HARDWARE_COMPONENT="${HARDWARE_COMPONENT:-Stm32H723ArmSystem}"
MICROROS_AGENT_SERVICE="${MICROROS_AGENT_SERVICE:-micro-ros-agent-stm32h723.service}"
REQUIRED_ACTIVE_JOINT_MASK=$((0x3F))
START_LEADER="false"
START_CONTROLLER="true"
POSITION_CONTROLLER="teleop"
ENABLE_GRAVITY_CONTROLLER="false"
ENABLE_JOINT4_CALIBRATION="false"
ENABLE_GRIPPER="false"
CLEAN_BEFORE_START="true"
DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
LEADER_STATE_TIMEOUT="90"
FOLLOWER_STATE_TIMEOUT="30"
STM32_STATE_TIMEOUT="30"
CONTROLLER_START_TIMEOUT="30"
RELAY_START_TIMEOUT="15"
LEADER_PID=""
CONTROLLER_PID=""
RELAY_PID=""
GAINS_FILE="$WORKSPACE_DIR/install/microros_stm32h723_bridge/share/microros_stm32h723_bridge/config/arm_mit_gains.yaml"
CURRENT_STAGE=0
TOTAL_STAGES=9
LAST_TOPIC_SAMPLE=""

usage() {
    echo "Usage: $0 [options]"
    echo
    echo "  --position-controller=teleop|trajectory|none  Select the position controller (default: teleop)."
    echo "  --gravity=true|false  Enable the independent effort controller (default: false)."
    echo "  --joint4-calibration=true|false  Run guarded joint4 calibration (default: false)."
    echo "  --gripper=true|false  Enable calibrated gripper teleoperation (default: false)."
    echo "  --mode=teleop|gravity|calibration  Backward-compatible preset alias."
    echo "  --start-leader=true   Start robo_driver in /leader namespace with lock disabled."
    echo "  --start-leader=false  Require an existing /leader/joint_states publisher (default)."
    echo "  --start-controller=true   Start ros2_control when it is not already running (default)."
    echo "  --start-controller=false  Require an existing Controller Manager."
    echo "  --clean=true           Stop stale project ROS 2 processes before startup (default)."
    echo "  --clean=false          Preserve existing processes and reuse running nodes."
    echo "  --ros-domain-id=N     Use an explicit ROS domain (default: current value or 0)."
    echo "  --leader-timeout=N    Wait N seconds for leader state (default: 90)."
    echo "  --follower-timeout=N  Wait N seconds for follower state (default: 30)."
    echo "  --stm32-timeout=N     Wait N seconds for STM32 feedback (default: 30)."
    echo "  --controller-timeout=N  Wait N seconds for controllers (default: 30)."
    echo "  --relay-timeout=N     Wait N seconds for relay output (default: 15)."
    echo "  --agent-service=NAME  micro-ROS Agent systemd unit name."
}

for argument in "$@"; do
    case "$argument" in
        --mode=*)
            case "${argument#*=}" in
                teleop)
                    POSITION_CONTROLLER="teleop"
                    ENABLE_GRAVITY_CONTROLLER="false"
                    ENABLE_JOINT4_CALIBRATION="false"
                    ;;
                gravity)
                    POSITION_CONTROLLER="none"
                    ENABLE_GRAVITY_CONTROLLER="true"
                    ENABLE_JOINT4_CALIBRATION="false"
                    ;;
                calibration)
                    POSITION_CONTROLLER="none"
                    ENABLE_GRAVITY_CONTROLLER="false"
                    ENABLE_JOINT4_CALIBRATION="true"
                    ;;
                *)
                    echo "ERROR: --mode must be teleop, gravity, or calibration." >&2
                    exit 2
                    ;;
            esac
            ;;
        --position-controller=*)
            POSITION_CONTROLLER="${argument#*=}"
            ;;
        --gravity=*)
            ENABLE_GRAVITY_CONTROLLER="${argument#*=}"
            ;;
        --joint4-calibration=*)
            ENABLE_JOINT4_CALIBRATION="${argument#*=}"
            ;;
        --gripper=*)
            ENABLE_GRIPPER="${argument#*=}"
            ;;
        --start-leader=true)
            START_LEADER="true"
            ;;
        --start-leader=false)
            START_LEADER="false"
            ;;
        --start-controller=true)
            START_CONTROLLER="true"
            ;;
        --start-controller=false)
            START_CONTROLLER="false"
            ;;
        --clean=true)
            CLEAN_BEFORE_START="true"
            ;;
        --clean=false)
            CLEAN_BEFORE_START="false"
            ;;
        --ros-domain-id=*)
            DOMAIN_ID="${argument#*=}"
            ;;
        --leader-timeout=*)
            LEADER_STATE_TIMEOUT="${argument#*=}"
            ;;
        --follower-timeout=*)
            FOLLOWER_STATE_TIMEOUT="${argument#*=}"
            ;;
        --stm32-timeout=*)
            STM32_STATE_TIMEOUT="${argument#*=}"
            ;;
        --controller-timeout=*)
            CONTROLLER_START_TIMEOUT="${argument#*=}"
            ;;
        --relay-timeout=*)
            RELAY_START_TIMEOUT="${argument#*=}"
            ;;
        --agent-service=*)
            MICROROS_AGENT_SERVICE="${argument#*=}"
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

if [[ "$POSITION_CONTROLLER" != "teleop" && \
    "$POSITION_CONTROLLER" != "trajectory" && \
    "$POSITION_CONTROLLER" != "none" ]]; then
    echo "ERROR: --position-controller must be teleop, trajectory, or none." >&2
    exit 2
fi
if [[ "$ENABLE_GRAVITY_CONTROLLER" != "true" && \
    "$ENABLE_GRAVITY_CONTROLLER" != "false" ]]; then
    echo "ERROR: --gravity must be true or false." >&2
    exit 2
fi
if [[ "$ENABLE_JOINT4_CALIBRATION" != "true" && \
    "$ENABLE_JOINT4_CALIBRATION" != "false" ]]; then
    echo "ERROR: --joint4-calibration must be true or false." >&2
    exit 2
fi
if [[ "$ENABLE_JOINT4_CALIBRATION" == "true" && \
    "$ENABLE_GRAVITY_CONTROLLER" == "true" ]]; then
    echo "ERROR: gravity and joint4 calibration controllers claim the same effort interfaces." >&2
    exit 2
fi
if [[ "$ENABLE_JOINT4_CALIBRATION" == "true" && \
    "$POSITION_CONTROLLER" != "none" ]]; then
    echo "ERROR: joint4 calibration requires --position-controller=none." >&2
    exit 2
fi
if [[ "$ENABLE_GRIPPER" != "true" && "$ENABLE_GRIPPER" != "false" ]]; then
    echo "ERROR: --gripper must be true or false." >&2
    exit 2
fi
if [[ "$ENABLE_GRIPPER" == "true" && "$POSITION_CONTROLLER" != "teleop" ]]; then
    echo "ERROR: Initial gripper testing requires --position-controller=teleop." >&2
    exit 2
fi
if [[ "$POSITION_CONTROLLER" != "teleop" ]]; then
    START_LEADER="false"
fi
if [[ "$POSITION_CONTROLLER" == "none" ]] && \
    { [[ "$ENABLE_GRAVITY_CONTROLLER" == "true" ]] || \
      [[ "$ENABLE_JOINT4_CALIBRATION" == "true" ]]; }; then
    GAINS_FILE="$WORKSPACE_DIR/install/microros_stm32h723_bridge/share/microros_stm32h723_bridge/config/gravity_mit_gains.yaml"
fi
if [[ "$ENABLE_GRIPPER" == "true" ]]; then
    REQUIRED_ACTIVE_JOINT_MASK=$((0x7F))
    GAINS_FILE="$WORKSPACE_DIR/install/microros_stm32h723_bridge/share/microros_stm32h723_bridge/config/arm_gripper_mit_gains.yaml"
fi
SELECTED_EFFORT_CONTROLLER=""
if [[ "$ENABLE_GRAVITY_CONTROLLER" == "true" ]]; then
    SELECTED_EFFORT_CONTROLLER="gravity_compensation_controller"
elif [[ "$ENABLE_JOINT4_CALIBRATION" == "true" ]]; then
    SELECTED_EFFORT_CONTROLLER="joint4_calibration_controller"
fi

if [[ ! "$DOMAIN_ID" =~ ^[0-9]+$ ]]; then
    echo "ERROR: --ros-domain-id must be a non-negative integer." >&2
    exit 2
fi
if [[ ! "$LEADER_STATE_TIMEOUT" =~ ^[1-9][0-9]*$ ]]; then
    echo "ERROR: --leader-timeout must be a positive integer." >&2
    exit 2
fi
if [[ ! "$CONTROLLER_START_TIMEOUT" =~ ^[1-9][0-9]*$ ]]; then
    echo "ERROR: --controller-timeout must be a positive integer." >&2
    exit 2
fi
if [[ ! "$FOLLOWER_STATE_TIMEOUT" =~ ^[1-9][0-9]*$ ]]; then
    echo "ERROR: --follower-timeout must be a positive integer." >&2
    exit 2
fi
if [[ ! "$STM32_STATE_TIMEOUT" =~ ^[1-9][0-9]*$ ]]; then
    echo "ERROR: --stm32-timeout must be a positive integer." >&2
    exit 2
fi
if [[ ! "$RELAY_START_TIMEOUT" =~ ^[1-9][0-9]*$ ]]; then
    echo "ERROR: --relay-timeout must be a positive integer." >&2
    exit 2
fi

if [[ ! -f "$ROS_SETUP" ]]; then
    echo "ERROR: ROS 2 Humble setup not found: $ROS_SETUP" >&2
    exit 1
fi
if [[ ! -f "$WORKSPACE_SETUP" ]]; then
    echo "ERROR: Workspace is not built: $WORKSPACE_SETUP" >&2
    exit 1
fi

source "$ROS_SETUP"
source "$WORKSPACE_SETUP"
export ROS_DOMAIN_ID="$DOMAIN_ID"
export RCUTILS_COLORIZED_OUTPUT=0
echo "Using ROS_DOMAIN_ID=$ROS_DOMAIN_ID"

stage() {
    CURRENT_STAGE=$((CURRENT_STAGE + 1))
    echo
    echo "[$CURRENT_STAGE/$TOTAL_STAGES] $1"
}

pass() {
    echo "[PASS] $1"
}

strip_ansi() {
    sed $'s/\033\[[0-9;]*m//g'
}

restart_ros2_daemon() {
    ros2 daemon stop >/dev/null 2>&1 || true
    sleep 1
    ros2 daemon start >/dev/null
    sleep 1
}

check_topic_once() {
    local topic="$1"
    local timeout_seconds="$2"
    local deadline=$((SECONDS + timeout_seconds))
    local attempt_timeout=10
    local topic_output=""

    LAST_TOPIC_SAMPLE=""
    while (( SECONDS < deadline )); do
        if (( deadline - SECONDS < attempt_timeout )); then
            attempt_timeout=$((deadline - SECONDS))
        fi
        if (( attempt_timeout < 1 )); then
            break
        fi

        if topic_output="$(
            timeout "$attempt_timeout" ros2 topic echo "$topic" --once 2>&1
        )"; then
            LAST_TOPIC_SAMPLE="$topic_output"
            return
        fi

        if grep -q '!rclpy.ok()' <<<"$topic_output"; then
            echo "ROS 2 CLI daemon context failed while checking $topic; restarting daemon."
            restart_ros2_daemon
        else
            sleep 1
        fi
        attempt_timeout=10
    done

    echo "ERROR: No message received from $topic within $timeout_seconds seconds." >&2
    if [[ -n "$topic_output" ]] && grep -q '!rclpy.ok()' <<<"$topic_output"; then
        echo "$topic_output" >&2
    fi
    return 1
}

controller_has_state() {
    local controller_name="$1"
    local expected_state="$2"
    local controller_output="$3"

    controller_output="$(strip_ansi <<<"$controller_output")"
    grep -Eq \
        "^${controller_name}[[:space:]].*[[:space:]]${expected_state}[[:space:]]*$" \
        <<<"$controller_output"
}

gripper_controller_matches() {
    local controller_output="$1"
    if [[ "$ENABLE_GRIPPER" == "true" ]]; then
        controller_has_state gripper_teleop_controller active "$controller_output"
    else
        ! grep -Eq '^gripper_teleop_controller[[:space:]]' \
            <<<"$(strip_ansi <<<"$controller_output")"
    fi
}

check_micro_ros_agent() {
    local service_environment=""

    if systemctl is-active --quiet "$MICROROS_AGENT_SERVICE" 2>/dev/null; then
        pass "systemd service $MICROROS_AGENT_SERVICE is active."
        service_environment="$(
            systemctl show "$MICROROS_AGENT_SERVICE" \
                --property=Environment --value 2>/dev/null || true
        )"
        if [[ "$service_environment" =~ ROS_DOMAIN_ID=([0-9]+) ]] && \
            [[ "${BASH_REMATCH[1]}" != "$ROS_DOMAIN_ID" ]]; then
            echo "ERROR: $MICROROS_AGENT_SERVICE uses ROS_DOMAIN_ID=${BASH_REMATCH[1]}," >&2
            echo "but this script uses ROS_DOMAIN_ID=$ROS_DOMAIN_ID." >&2
            return 1
        fi
        return
    fi

    if pgrep -f '/micro_ros_agent/micro_ros_agent serial' >/dev/null 2>&1; then
        pass "A manually managed micro-ROS Agent serial process is running."
        return
    fi

    echo "ERROR: micro-ROS Agent is not running." >&2
    echo "Check: systemctl status $MICROROS_AGENT_SERVICE" >&2
    return 1
}

check_stm32_status() {
    local status_sample="$1"
    local allow_disabled_stale_sequence="${2:-false}"
    local active_mask=""
    local fault_code=""
    local mode=""

    active_mask="$(awk '/^active_joint_mask:/ {print $2; exit}' <<<"$status_sample")"
    fault_code="$(awk '/^fault_code:/ {print $2; exit}' <<<"$status_sample")"
    mode="$(awk '/^mode:/ {print $2; exit}' <<<"$status_sample")"
    if [[ ! "$active_mask" =~ ^[0-9]+$ ]]; then
        echo "ERROR: STM32 active_joint_mask is not a decimal integer: '$active_mask'." >&2
        return 1
    fi
    if (( (active_mask & REQUIRED_ACTIVE_JOINT_MASK) != REQUIRED_ACTIVE_JOINT_MASK )); then
        printf 'ERROR: STM32 active_joint_mask is %s (0x%02X); required bits 0x%02X are not all active.\n' \
            "$active_mask" "$active_mask" "$REQUIRED_ACTIVE_JOINT_MASK" >&2
        return 1
    fi
    if [[ "$fault_code" != "0" ]]; then
        if [[ "$allow_disabled_stale_sequence" == "true" && \
            "$fault_code" == "4" && "$mode" == "3" ]]; then
            echo "[WARN] STM32 reports recoverable STALE_SEQUENCE while disabled."
            echo "       MicroRosSystem will resume after last_command_sequence."
            return
        fi
        case "$fault_code" in
            11)
                echo "ERROR: STM32 fault_code=11 (COMMAND_LOSS_LATCHED)." >&2
                echo "Reset or power-cycle the STM32 to clear this latched fault." >&2
                ;;
            *)
                echo "ERROR: STM32 fault_code is '$fault_code', expected 0." >&2
                ;;
        esac
        return 1
    fi
    printf '[PASS] STM32 active_joint_mask=%s (0x%02X) contains required mask 0x%02X; fault_code=0.\n' \
        "$active_mask" "$active_mask" "$REQUIRED_ACTIVE_JOINT_MASK"
}

safe_stop_ros2_control() {
    local response=""
    local status_sample=""
    local stop_deadline=0
    local status_mode=""

    if ! pgrep -f '/controller_manager/ros2_control_node' >/dev/null 2>&1; then
        return 0
    fi

    response="$(
        timeout 8 ros2 service call \
            /microros_stm32h723_hardware/safe_stop \
            std_srvs/srv/Trigger '{}' 2>/dev/null || true
    )"
    if ! grep -Eq 'success=(True|true)|success:[[:space:]]*true' <<<"$response"; then
        echo "WARNING: Hardware safe-stop was unavailable or rejected." >&2
        return 1
    fi

    echo "Waiting for STM32 MODE_DISABLE confirmation."
    stop_deadline=$((SECONDS + 10))
    while (( SECONDS < stop_deadline )); do
        status_sample="$(
            timeout 3 ros2 topic echo /stm32/status microros_stm32h723_bridge/msg/HardwareStatus --once --no-daemon 2>/dev/null || true
        )"
        status_mode="$(awk '/^mode:/ {print $2; exit}' <<<"$status_sample")"
        if [[ "$status_mode" == "3" ]]; then
            pass "STM32 confirmed MODE_DISABLE before ros2_control shutdown."
            return 0
        fi
        sleep 1
    done
    echo "WARNING: STM32 did not confirm MODE_DISABLE before cleanup." >&2
    return 1
}

stop_matching_processes() {
    local description="$1"
    local pattern="$2"

    if ! pgrep -f -- "$pattern" >/dev/null 2>&1; then
        return 0
    fi

    echo "Stopping stale $description process(es)."
    pkill -INT -f -- "$pattern" 2>/dev/null || true
    for _ in {1..20}; do
        if ! pgrep -f -- "$pattern" >/dev/null 2>&1; then
            return 0
        fi
        sleep 0.1
    done

    pkill -TERM -f -- "$pattern" 2>/dev/null || true
    sleep 1
    if pgrep -f -- "$pattern" >/dev/null 2>&1; then
        echo "Force stopping stale $description process(es)."
        pkill -KILL -f -- "$pattern" 2>/dev/null || true
    fi
}

stop_process_group() {
    local description="$1"
    local group_leader_pid="$2"

    if [[ -z "$group_leader_pid" ]] || \
        ! kill -0 -- "-$group_leader_pid" 2>/dev/null; then
        return 0
    fi

    echo "Stopping $description process group (PGID $group_leader_pid)."
    kill -INT -- "-$group_leader_pid" 2>/dev/null || true
    for _ in {1..30}; do
        if ! kill -0 -- "-$group_leader_pid" 2>/dev/null; then
            wait "$group_leader_pid" 2>/dev/null || true
            return 0
        fi
        sleep 0.1
    done

    kill -TERM -- "-$group_leader_pid" 2>/dev/null || true
    sleep 1
    if kill -0 -- "-$group_leader_pid" 2>/dev/null; then
        echo "Force stopping $description process group."
        kill -KILL -- "-$group_leader_pid" 2>/dev/null || true
    fi
    wait "$group_leader_pid" 2>/dev/null || true
}

cleanup() {
    trap - EXIT
    # wait() may propagate a child's SIGINT to this shell. Keep it ignored
    # throughout cleanup so a relay exit cannot skip the leader/controller.
    trap '' INT TERM
    # A relay that exits with status 130 must not abort the remaining cleanup.
    set +e
    if [[ -n "$CONTROLLER_PID" ]] && ! safe_stop_ros2_control; then
        echo "ERROR: Refusing cleanup without DISABLE confirmation; command processes are preserved." >&2
        return 1
    fi
    if [[ -n "$RELAY_PID" ]]; then
        echo "Stopping teleoperation relay started by this script."
        stop_process_group "teleoperation relay" "$RELAY_PID"
        stop_matching_processes \
            "arm relay launch" \
            "ros2 launch joint_state_relay relay_to_microros_arm.launch.py"
        stop_matching_processes "joint_state_relay node" "/joint_state_relay/relay_node"
    fi
    if [[ -n "$LEADER_PID" ]]; then
        echo "Stopping leader robo_driver started by this script."
        stop_process_group "leader robo_driver" "$LEADER_PID"
        stop_matching_processes "leader robo_driver executable" "/robo_driver/driver"
        stop_matching_processes "leader ros2 run wrapper" "ros2 run robo_driver driver"
    fi
    if [[ -n "$CONTROLLER_PID" ]]; then
        echo "Stopping ros2_control launch started by this script."
        stop_process_group "arm ros2_control" "$CONTROLLER_PID"
        stop_matching_processes \
            "arm ros2_control launch" \
            "ros2 launch microros_stm32h723_bridge stm32h723_arm.launch.py"
        stop_matching_processes "ros2_control_node" "/controller_manager/ros2_control_node"
        stop_matching_processes \
            "follower robot_state_publisher" \
            "/robot_state_publisher/robot_state_publisher"
    fi
}
trap cleanup EXIT INT TERM

stage "ROS environment and stale-process cleanup"
pass "ROS 2 Humble and rebotarm-ros2-stack overlay are sourced (domain $ROS_DOMAIN_ID)."
if [[ "$CLEAN_BEFORE_START" == "true" ]]; then
    echo "Cleaning stale arm teleoperation processes."
    stop_matching_processes \
        "arm relay launch" \
        "ros2 launch joint_state_relay relay_to_microros_arm.launch.py"
    stop_matching_processes \
        "joint_state_relay node" \
        "/joint_state_relay/relay_node"

    if [[ "$START_LEADER" == "true" ]]; then
        stop_matching_processes "leader robo_driver" "/robo_driver/driver"
        stop_matching_processes "leader ros2 run wrapper" "ros2 run robo_driver driver"
    fi

    if [[ "$START_CONTROLLER" == "true" ]]; then
        safe_stop_ros2_control
        stop_matching_processes \
            "arm ros2_control launch" \
            "ros2 launch microros_stm32h723_bridge stm32h723_arm.launch.py"
        stop_matching_processes "ros2_control_node" "/controller_manager/ros2_control_node"
        stop_matching_processes "controller spawner" "/controller_manager/spawner"
        stop_matching_processes \
            "follower robot_state_publisher" \
            "/robot_state_publisher/robot_state_publisher"
    fi

    echo "Restarting the ROS 2 daemon for domain $ROS_DOMAIN_ID."
    restart_ros2_daemon
    pass "Stale project processes were stopped and the ROS 2 daemon was restarted."
else
    pass "Cleanup disabled; existing ROS 2 processes are preserved."
fi

stage "Leader arm driver"
if [[ "$POSITION_CONTROLLER" != "teleop" ]]; then
    pass "Position controller is $POSITION_CONTROLLER: leader driver is not started."
elif [[ "$START_LEADER" == "true" ]]; then
    if [[ ! -f "$LEADER_SETUP" ]]; then
        echo "ERROR: Leader workspace is not built: $LEADER_SETUP" >&2
        exit 1
    fi
    if [[ ! -e /dev/ttyCH341USB0 ]]; then
        echo "ERROR: Leader serial device not found: /dev/ttyCH341USB0" >&2
        exit 1
    fi

    echo "Starting leader robo_driver on /dev/ttyCH341USB0."
    echo "Leader torque lock is disabled for manual teleoperation."
    setsid bash -c '
        set -e
        source "$1"
        source "$2"
        export ROS_DOMAIN_ID="$3"
        export RCUTILS_COLORIZED_OUTPUT=0
        exec ros2 run robo_driver driver --ros-args \
            -r __ns:=/leader \
            -p lock:=disable
    ' _ "$ROS_SETUP" "$LEADER_SETUP" "$ROS_DOMAIN_ID" &
    LEADER_PID=$!
    pass "Leader robo_driver process started (PID $LEADER_PID)."
else
    pass "Using an externally managed leader driver."
fi

stage "Leader arm ROS feedback"
if [[ "$POSITION_CONTROLLER" != "teleop" ]]; then
    pass "Position controller is $POSITION_CONTROLLER: leader feedback check is skipped."
elif ! check_topic_once /leader/joint_states "$LEADER_STATE_TIMEOUT"; then
    echo "Check the same domain explicitly:" >&2
    echo "  ROS_DOMAIN_ID=$ROS_DOMAIN_ID ros2 topic echo /leader/joint_states --once" >&2
    exit 1
fi
if [[ "$POSITION_CONTROLLER" == "teleop" ]]; then
    pass "/leader/joint_states is publishing."
fi

stage "Micro-ROS Agent and STM32 endpoint"
check_micro_ros_agent
check_topic_once /stm32/status "$STM32_STATE_TIMEOUT"
stm32_status_sample="$LAST_TOPIC_SAMPLE"
check_stm32_status "$stm32_status_sample" true
check_topic_once /stm32/joint_states "$STM32_STATE_TIMEOUT"
pass "/stm32/joint_states is publishing."

stage "Controller Manager and controller states"
if pgrep -f '/controller_manager/ros2_control_node' >/dev/null 2>&1; then
    echo "Using the existing ros2_control_node process."
elif [[ "$START_CONTROLLER" == "true" ]]; then
    echo "Starting STM32H723 arm ros2_control (gripper=$ENABLE_GRIPPER, calibration=$ENABLE_JOINT4_CALIBRATION)."
    setsid ros2 launch microros_stm32h723_bridge \
        stm32h723_arm.launch.py \
        position_controller:="$POSITION_CONTROLLER" \
        enable_gravity_controller:="$ENABLE_GRAVITY_CONTROLLER" \
        enable_joint4_calibration:="$ENABLE_JOINT4_CALIBRATION" \
        enable_gripper:="$ENABLE_GRIPPER" \
        gains_file:="$GAINS_FILE" &
    CONTROLLER_PID=$!
else
    echo "ERROR: ros2_control_node is not running." >&2
    echo "Start it manually or use --start-controller=true." >&2
    exit 1
fi

echo "Waiting for Controller Manager and selected arm controllers."
if [[ "$POSITION_CONTROLLER" == "none" && -n "$SELECTED_EFFORT_CONTROLLER" ]]; then
    echo "Waiting for effort controllers to load before hardware activation."
    gravity_load_deadline=$((SECONDS + CONTROLLER_START_TIMEOUT))
    gravity_load_list=""
    while (( SECONDS < gravity_load_deadline )); do
        gravity_load_list="$(
            timeout 5 ros2 control list_controllers \
                --controller-manager "$CONTROLLER_MANAGER" 2>&1 || true
        )"
        if controller_has_state joint_state_broadcaster inactive "$gravity_load_list" && \
            controller_has_state "$SELECTED_EFFORT_CONTROLLER" inactive "$gravity_load_list"; then
            break
        fi
        sleep 1
    done
    if ! controller_has_state joint_state_broadcaster inactive "$gravity_load_list" || \
        ! controller_has_state "$SELECTED_EFFORT_CONTROLLER" inactive "$gravity_load_list"; then
        echo "$gravity_load_list" >&2
        echo "ERROR: effort controllers were not loaded inactive before hardware activation." >&2
        exit 1
    fi
    timeout "$CONTROLLER_START_TIMEOUT" ros2 control set_hardware_component_state \
        "$HARDWARE_COMPONENT" active \
        --controller-manager "$CONTROLLER_MANAGER"
    timeout 10 ros2 control switch_controllers \
        --activate joint_state_broadcaster "$SELECTED_EFFORT_CONTROLLER" \
        --strict --controller-manager "$CONTROLLER_MANAGER"
fi
TELEOP_STATE="inactive"
TRAJECTORY_STATE="inactive"
GRAVITY_STATE="inactive"
CALIBRATION_STATE="inactive"
ACTIVE_COMMAND_CONTROLLERS=()
if [[ "$POSITION_CONTROLLER" == "teleop" ]]; then
    TELEOP_STATE="active"
    ACTIVE_COMMAND_CONTROLLERS+=(arm_teleop_controller)
elif [[ "$POSITION_CONTROLLER" == "trajectory" ]]; then
    TRAJECTORY_STATE="active"
    ACTIVE_COMMAND_CONTROLLERS+=(arm_trajectory_controller)
fi
if [[ "$ENABLE_GRAVITY_CONTROLLER" == "true" ]]; then
    GRAVITY_STATE="active"
    ACTIVE_COMMAND_CONTROLLERS+=(gravity_compensation_controller)
fi
if [[ "$ENABLE_JOINT4_CALIBRATION" == "true" ]]; then
    CALIBRATION_STATE="active"
    ACTIVE_COMMAND_CONTROLLERS+=(joint4_calibration_controller)
fi
if [[ "$ENABLE_GRIPPER" == "true" ]]; then
    ACTIVE_COMMAND_CONTROLLERS+=(gripper_teleop_controller)
fi
controller_deadline=$((SECONDS + CONTROLLER_START_TIMEOUT))
controller_list=""
while (( SECONDS < controller_deadline )); do
    if controller_list="$(
            timeout 5 ros2 control list_controllers \
                --controller-manager "$CONTROLLER_MANAGER" 2>&1
        )" && \
        controller_has_state joint_state_broadcaster active "$controller_list" && \
        { [[ "$POSITION_CONTROLLER" == "none" ]] || \
          controller_has_state arm_teleop_controller "$TELEOP_STATE" "$controller_list"; } && \
        { [[ "$POSITION_CONTROLLER" == "none" ]] || \
          controller_has_state arm_trajectory_controller "$TRAJECTORY_STATE" "$controller_list"; } && \
        controller_has_state gravity_compensation_controller "$GRAVITY_STATE" "$controller_list" && \
        controller_has_state joint4_calibration_controller "$CALIBRATION_STATE" "$controller_list" && \
        gripper_controller_matches "$controller_list"; then
        break
    fi
    sleep 1
done
controller_list="$(strip_ansi <<<"$controller_list")"
if ! controller_has_state joint_state_broadcaster active "$controller_list" || \
    { [[ "$POSITION_CONTROLLER" != "none" ]] && \
      ! controller_has_state arm_teleop_controller "$TELEOP_STATE" "$controller_list"; } || \
    { [[ "$POSITION_CONTROLLER" != "none" ]] && \
      ! controller_has_state arm_trajectory_controller "$TRAJECTORY_STATE" "$controller_list"; } || \
    ! controller_has_state gravity_compensation_controller "$GRAVITY_STATE" "$controller_list" || \
    ! controller_has_state joint4_calibration_controller "$CALIBRATION_STATE" "$controller_list" || \
    ! gripper_controller_matches "$controller_list"; then
    echo "$controller_list" >&2
    echo "ERROR: selected arm controllers were not ready within $CONTROLLER_START_TIMEOUT seconds." >&2
    echo "If the output contains !rclpy.ok(), run: ros2 daemon stop && ros2 daemon start" >&2
    exit 1
fi
echo "$controller_list"
pass "Controllers match position=$POSITION_CONTROLLER, gravity=$ENABLE_GRAVITY_CONTROLLER, calibration=$ENABLE_JOINT4_CALIBRATION, gripper=$ENABLE_GRIPPER."

stage "MicroRosSystem HardwareInterface"
echo "Checking ros2_control hardware lifecycle."
hardware_components="$(
    timeout 10 ros2 control list_hardware_components \
        --controller-manager "$CONTROLLER_MANAGER" 2>&1 | strip_ansi || true
)"
if ! grep -Eq 'label=active' <<<"$hardware_components"; then
    echo "HardwareInterface is not active; recovering its lifecycle."
    for controller in "${ACTIVE_COMMAND_CONTROLLERS[@]}"; do
        timeout 10 ros2 control set_controller_state \
            "$controller" inactive \
            --controller-manager "$CONTROLLER_MANAGER" >/dev/null 2>&1 || true
    done
    timeout 10 ros2 control set_controller_state \
        joint_state_broadcaster inactive \
        --controller-manager "$CONTROLLER_MANAGER" >/dev/null 2>&1 || true
    timeout "$CONTROLLER_START_TIMEOUT" ros2 control set_hardware_component_state \
        "$HARDWARE_COMPONENT" active \
        --controller-manager "$CONTROLLER_MANAGER"
    timeout 10 ros2 control set_controller_state \
        joint_state_broadcaster active \
        --controller-manager "$CONTROLLER_MANAGER"
    for controller in "${ACTIVE_COMMAND_CONTROLLERS[@]}"; do
        timeout 10 ros2 control set_controller_state \
            "$controller" active \
            --controller-manager "$CONTROLLER_MANAGER"
    done
fi

hardware_components="$(
    timeout 10 ros2 control list_hardware_components \
        --controller-manager "$CONTROLLER_MANAGER" 2>&1 | strip_ansi || true
)"
hardware_interfaces="$(
    timeout 10 ros2 control list_hardware_interfaces \
        --controller-manager "$CONTROLLER_MANAGER" 2>&1 | strip_ansi || true
)"
interfaces_valid="true"
for joint_index in {1..6}; do
    if [[ "$POSITION_CONTROLLER" != "none" ]] && \
        ! grep -Eq "joint${joint_index}/position \[available\] \[claimed\]" <<<"$hardware_interfaces"; then
        interfaces_valid="false"
    fi
    if [[ -n "$SELECTED_EFFORT_CONTROLLER" ]] && \
        ! grep -Eq "joint${joint_index}/effort \[available\] \[claimed\]" <<<"$hardware_interfaces"; then
        interfaces_valid="false"
    fi
done
if [[ "$ENABLE_GRIPPER" == "true" ]] && \
    ! grep -Eq "gripper_finger_left_joint/position \[available\] \[claimed\]" \
        <<<"$hardware_interfaces"; then
    interfaces_valid="false"
fi
if ! grep -Eq 'label=active' <<<"$hardware_components" || [[ "$interfaces_valid" != "true" ]]; then
    echo "$hardware_components" >&2
    echo "$hardware_interfaces" >&2
    echo "ERROR: HardwareInterface command-interface claims do not match the selected controllers." >&2
    exit 1
fi
pass "HardwareInterface claims match the selected arm, gravity, and gripper controllers."

stage "Follower joint state from ros2_control"
check_topic_once /joint_states "$FOLLOWER_STATE_TIMEOUT"
follower_state_sample="$LAST_TOPIC_SAMPLE"
if [[ "$ENABLE_GRIPPER" == "true" ]] && \
    ! grep -q 'gripper_finger_left_joint' <<<"$follower_state_sample"; then
    echo "ERROR: /joint_states does not contain calibrated gripper feedback." >&2
    exit 1
fi
pass "/joint_states is publishing the selected follower feedback."

stage "Leader-to-follower relay"
if [[ "$POSITION_CONTROLLER" != "teleop" ]]; then
    pass "Position controller is $POSITION_CONTROLLER: leader relay is not started."
else
    echo "Starting joint1-6 teleoperation relay."
    echo "Publishing: /arm_teleop_controller/commands"

    setsid ros2 launch joint_state_relay \
        relay_to_microros_arm.launch.py &
    RELAY_PID=$!
    if ! check_topic_once \
        /arm_teleop_controller/commands "$RELAY_START_TIMEOUT"; then
        if ! kill -0 "$RELAY_PID" 2>/dev/null; then
            wait "$RELAY_PID" 2>/dev/null || true
            echo "ERROR: joint_state_relay exited during startup." >&2
        else
            echo "ERROR: relay is running but did not publish a command." >&2
            echo "Check leader and follower timestamps and relative alignment logs." >&2
        fi
        exit 1
    fi
    echo "Relay output is healthy: /arm_teleop_controller/commands"
    pass "Relay is publishing position commands for joint1-6."
    if [[ "$ENABLE_GRIPPER" == "true" ]]; then
        check_topic_once \
            /gripper_teleop_controller/commands "$RELAY_START_TIMEOUT"
        pass "Relay is publishing calibrated, rate-limited gripper commands."
    fi
fi

stage "Micro-ROS bridge command and acknowledgement"
check_topic_once /stm32/joint_commands "$RELAY_START_TIMEOUT"
pass "MicroRosSystem is publishing /stm32/joint_commands."

subscriber_deadline=$((SECONDS + RELAY_START_TIMEOUT))
stm32_command_info=""
stm32_subscription_count="0"
while (( SECONDS < subscriber_deadline )); do
    stm32_command_info="$(
        timeout 5 ros2 topic info /stm32/joint_commands --no-daemon --spin-time 1 2>/dev/null || true
    )"
    stm32_subscription_count="$(
        awk -F': ' '/^Subscription count:/ {print $2; exit}' <<<"$stm32_command_info"
    )"
    if [[ "$stm32_subscription_count" =~ ^[1-9][0-9]*$ ]]; then
        break
    fi
    sleep 0.5
done
if [[ ! "$stm32_subscription_count" =~ ^[1-9][0-9]*$ ]]; then
    echo "$stm32_command_info" >&2
    echo "ERROR: /stm32/joint_commands has no STM32/micro-ROS subscriber after discovery timeout." >&2
    exit 1
fi
pass "/stm32/joint_commands has $stm32_subscription_count subscriber(s)."

ack_deadline=$((SECONDS + STM32_STATE_TIMEOUT))
last_command_sequence="0"
while (( SECONDS < ack_deadline )); do
    stm32_status_sample="$(
        timeout 5 ros2 topic echo /stm32/status microros_stm32h723_bridge/msg/HardwareStatus --once --no-daemon 2>/dev/null || true
    )"
    last_command_sequence="$(
        awk '/^last_command_sequence:/ {print $2; exit}' <<<"$stm32_status_sample"
    )"
    if [[ "$last_command_sequence" =~ ^[1-9][0-9]*$ ]]; then
        break
    fi
    sleep 1
done
if [[ ! "$last_command_sequence" =~ ^[1-9][0-9]*$ ]]; then
    echo "ERROR: STM32 did not acknowledge a non-zero command sequence." >&2
    exit 1
fi
check_stm32_status "$stm32_status_sample"
pass "STM32 acknowledged command sequence $last_command_sequence."

# Soft collection preflight (stamps / TRACK / fault). Hard gate is
# scripts/check_collection_ready.sh before recording (see docs/data-collection.md).
collection_check="$WORKSPACE_DIR/scripts/check_collection_ready.sh"
if [[ -x "$collection_check" ]]; then
    echo
    echo "Running collection stamp preflight (--warn-only; desk should hard-gate before record)."
    "$collection_check" --warn-only \
        --ros-domain-id="$ROS_DOMAIN_ID" \
        --timeout="$STM32_STATE_TIMEOUT" \
        --workspace="$WORKSPACE_DIR" || true
else
    echo "[WARN] Missing $collection_check; skip collection stamp preflight."
fi

echo
if [[ "$POSITION_CONTROLLER" == "teleop" ]]; then
    echo "End-to-end self-check passed:"
    echo "  leader arm -> /leader/joint_states"
    echo "  -> joint_state_relay -> /arm_teleop_controller/commands"
    if [[ "$ENABLE_GRAVITY_CONTROLLER" == "true" ]]; then
        echo "  + gravity_compensation_controller -> effort interfaces"
    fi
    if [[ "$ENABLE_GRIPPER" == "true" ]]; then
        echo "  + joint7_left -> /gripper_teleop_controller/commands"
    fi
    echo "  -> ros2_control MicroRosSystem -> /stm32/joint_commands"
    echo "  -> micro-ROS Agent -> STM32 status acknowledgement"
    wait "$RELAY_PID"
else
    echo "Controller self-check passed:"
    echo "  position_controller=$POSITION_CONTROLLER"
    echo "  gravity_controller=$ENABLE_GRAVITY_CONTROLLER"
    echo "  joint4_calibration_controller=$ENABLE_JOINT4_CALIBRATION"
    echo "  -> ros2_control MicroRosSystem -> /stm32/joint_commands"
    echo "  -> micro-ROS Agent -> STM32 status acknowledgement"
    wait "$CONTROLLER_PID"
fi
