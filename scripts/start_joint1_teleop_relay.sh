#!/usr/bin/env bash
# Start and verify the joint1-6 leader-to-follower ros2_control pipeline.
set -eo pipefail

WORKSPACE_DIR="${LEADER_FOLLOWER_WS:-$HOME/Project/leader_follower}"
LEADER_WORKSPACE_DIR="${LEADER_ARM_WS:-$HOME/Project/Star-Arm-102/ROS2_HUMBLE}"
ROS_SETUP="/opt/ros/humble/setup.bash"
WORKSPACE_SETUP="$WORKSPACE_DIR/install/setup.bash"
LEADER_SETUP="$LEADER_WORKSPACE_DIR/install/setup.bash"
CONTROLLER_MANAGER="${CONTROLLER_MANAGER:-/controller_manager}"
HARDWARE_COMPONENT="${HARDWARE_COMPONENT:-Stm32H723SingleJointSystem}"
MICROROS_AGENT_SERVICE="${MICROROS_AGENT_SERVICE:-micro-ros-agent-stm32h723.service}"
REQUIRED_ACTIVE_JOINT_MASK=$((0x3F))
START_LEADER="false"
START_CONTROLLER="true"
RUN_MODE="teleop"
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
GAINS_FILE="$WORKSPACE_DIR/install/microros_stm32h723_bridge/share/microros_stm32h723_bridge/config/single_joint_mit_gains.yaml"
CURRENT_STAGE=0
TOTAL_STAGES=9
LAST_TOPIC_SAMPLE=""

usage() {
    echo "Usage: $0 [options]"
    echo
    echo "  --mode=teleop|gravity  Full leader teleoperation or gravity-only tuning (default: teleop)."
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
            RUN_MODE="${argument#*=}"
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

if [[ "$RUN_MODE" != "teleop" && "$RUN_MODE" != "gravity" ]]; then
    echo "ERROR: --mode must be teleop or gravity." >&2
    exit 2
fi
if [[ "$RUN_MODE" == "gravity" ]]; then
    START_LEADER="false"
    GAINS_FILE="$WORKSPACE_DIR/install/microros_stm32h723_bridge/share/microros_stm32h723_bridge/config/gravity_mit_gains.yaml"
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
    local active_mask=""
    local fault_code=""

    active_mask="$(awk '/^active_joint_mask:/ {print $2; exit}' <<<"$status_sample")"
    fault_code="$(awk '/^fault_code:/ {print $2; exit}' <<<"$status_sample")"
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
    printf '[PASS] STM32 active_joint_mask=%s (0x%02X) contains required joint1-6 mask 0x%02X; fault_code=0.\n' \
        "$active_mask" "$active_mask" "$REQUIRED_ACTIVE_JOINT_MASK"
}

safe_stop_ros2_control() {
    local response=""
    local status_sample=""
    local stop_deadline=0
    local status_mode=""

    if ! pgrep -f '/controller_manager/ros2_control_node' >/dev/null 2>&1; then
        return
    fi

    response="$(
        timeout 8 ros2 service call \
            /microros_stm32h723_hardware/safe_stop \
            std_srvs/srv/Trigger '{}' 2>/dev/null || true
    )"
    if ! grep -Eq 'success=(True|true)|success:[[:space:]]*true' <<<"$response"; then
        echo "WARNING: Hardware safe-stop was unavailable or rejected." >&2
        return
    fi

    echo "Waiting for STM32 MODE_DISABLE confirmation."
    stop_deadline=$((SECONDS + 10))
    while (( SECONDS < stop_deadline )); do
        status_sample="$(
            timeout 3 ros2 topic echo /stm32/status --once 2>/dev/null || true
        )"
        status_mode="$(awk '/^mode:/ {print $2; exit}' <<<"$status_sample")"
        if [[ "$status_mode" == "3" ]]; then
            pass "STM32 confirmed MODE_DISABLE before ros2_control shutdown."
            return
        fi
        sleep 1
    done
    echo "WARNING: STM32 did not confirm MODE_DISABLE before cleanup." >&2
}

stop_matching_processes() {
    local description="$1"
    local pattern="$2"

    if ! pgrep -f -- "$pattern" >/dev/null 2>&1; then
        return
    fi

    echo "Stopping stale $description process(es)."
    pkill -INT -f -- "$pattern" 2>/dev/null || true
    for _ in {1..20}; do
        if ! pgrep -f -- "$pattern" >/dev/null 2>&1; then
            return
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
        return
    fi

    echo "Stopping $description process group (PGID $group_leader_pid)."
    kill -INT -- "-$group_leader_pid" 2>/dev/null || true
    for _ in {1..30}; do
        if ! kill -0 -- "-$group_leader_pid" 2>/dev/null; then
            wait "$group_leader_pid" 2>/dev/null || true
            return
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
    trap - EXIT INT TERM
    if [[ -n "$RELAY_PID" ]]; then
        echo "Stopping teleoperation relay started by this script."
        stop_process_group "teleoperation relay" "$RELAY_PID"
        stop_matching_processes \
            "joint1 relay launch" \
            "ros2 launch joint_state_relay relay_to_microros_single_joint.launch.py"
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
        safe_stop_ros2_control
        stop_process_group "single-joint ros2_control" "$CONTROLLER_PID"
        stop_matching_processes \
            "single-joint ros2_control launch" \
            "ros2 launch microros_stm32h723_bridge stm32h723_single_joint.launch.py"
        stop_matching_processes "ros2_control_node" "/controller_manager/ros2_control_node"
        stop_matching_processes \
            "follower robot_state_publisher" \
            "/robot_state_publisher/robot_state_publisher"
    fi
}
trap cleanup EXIT INT TERM

stage "ROS environment and stale-process cleanup"
pass "ROS 2 Humble and leader_follower overlay are sourced (domain $ROS_DOMAIN_ID)."
if [[ "$CLEAN_BEFORE_START" == "true" ]]; then
    echo "Cleaning stale joint1 teleoperation processes."
    stop_matching_processes \
        "joint1 relay launch" \
        "ros2 launch joint_state_relay relay_to_microros_single_joint.launch.py"
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
            "single-joint ros2_control launch" \
            "ros2 launch microros_stm32h723_bridge stm32h723_single_joint.launch.py"
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
if [[ "$RUN_MODE" == "gravity" ]]; then
    pass "Gravity-only mode: leader driver is not started."
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
if [[ "$RUN_MODE" == "gravity" ]]; then
    pass "Gravity-only mode: leader feedback check is skipped."
elif ! check_topic_once /leader/joint_states "$LEADER_STATE_TIMEOUT"; then
    echo "Check the same domain explicitly:" >&2
    echo "  ROS_DOMAIN_ID=$ROS_DOMAIN_ID ros2 topic echo /leader/joint_states --once" >&2
    exit 1
fi
if [[ "$RUN_MODE" == "teleop" ]]; then
    pass "/leader/joint_states is publishing."
fi

stage "Micro-ROS Agent and STM32 endpoint"
check_micro_ros_agent
check_topic_once /stm32/status "$STM32_STATE_TIMEOUT"
stm32_status_sample="$LAST_TOPIC_SAMPLE"
check_stm32_status "$stm32_status_sample"
check_topic_once /stm32/joint_states "$STM32_STATE_TIMEOUT"
pass "/stm32/joint_states is publishing."

stage "Controller Manager and controller states"
if pgrep -f '/controller_manager/ros2_control_node' >/dev/null 2>&1; then
    echo "Using the existing ros2_control_node process."
elif [[ "$START_CONTROLLER" == "true" ]]; then
    echo "Starting STM32H723 joint1-6 ros2_control."
    setsid ros2 launch microros_stm32h723_bridge \
        stm32h723_single_joint.launch.py controller_mode:="$RUN_MODE" \
        gains_file:="$GAINS_FILE" &
    CONTROLLER_PID=$!
else
    echo "ERROR: ros2_control_node is not running." >&2
    echo "Start it manually or use --start-controller=true." >&2
    exit 1
fi

echo "Waiting for Controller Manager and joint1-6 controllers."
ACTIVE_COMMAND_CONTROLLER="joint1_teleop_controller"
if [[ "$RUN_MODE" == "gravity" ]]; then
    ACTIVE_COMMAND_CONTROLLER="gravity_compensation_controller"
fi
controller_deadline=$((SECONDS + CONTROLLER_START_TIMEOUT))
controller_list=""
while (( SECONDS < controller_deadline )); do
    if controller_list="$(
            timeout 5 ros2 control list_controllers \
                --controller-manager "$CONTROLLER_MANAGER" 2>&1
        )" && \
        controller_has_state "$ACTIVE_COMMAND_CONTROLLER" active "$controller_list" && \
        controller_has_state joint_state_broadcaster active "$controller_list" && \
        controller_has_state joint1_trajectory_controller inactive "$controller_list"; then
        break
    fi
    sleep 1
done
controller_list="$(strip_ansi <<<"$controller_list")"
if ! controller_has_state "$ACTIVE_COMMAND_CONTROLLER" active "$controller_list" || \
    ! controller_has_state joint_state_broadcaster active "$controller_list" || \
    ! controller_has_state joint1_trajectory_controller inactive "$controller_list"; then
    echo "$controller_list" >&2
    echo "ERROR: joint1-6 controllers were not ready within $CONTROLLER_START_TIMEOUT seconds." >&2
    echo "If the output contains !rclpy.ok(), run: ros2 daemon stop && ros2 daemon start" >&2
    exit 1
fi
echo "$controller_list"
pass "joint_state_broadcaster and $ACTIVE_COMMAND_CONTROLLER are active."

stage "MicroRosSystem HardwareInterface"
echo "Checking ros2_control hardware lifecycle."
hardware_components="$(
    timeout 10 ros2 control list_hardware_components \
        --controller-manager "$CONTROLLER_MANAGER" 2>&1 | strip_ansi || true
)"
if ! grep -Eq 'label=active' <<<"$hardware_components"; then
    echo "HardwareInterface is not active; recovering its lifecycle."
    timeout 10 ros2 control set_controller_state \
        "$ACTIVE_COMMAND_CONTROLLER" inactive \
        --controller-manager "$CONTROLLER_MANAGER" >/dev/null 2>&1 || true
    timeout 10 ros2 control set_controller_state \
        joint_state_broadcaster inactive \
        --controller-manager "$CONTROLLER_MANAGER" >/dev/null 2>&1 || true
    timeout "$CONTROLLER_START_TIMEOUT" ros2 control set_hardware_component_state \
        "$HARDWARE_COMPONENT" active \
        --controller-manager "$CONTROLLER_MANAGER"
    timeout 10 ros2 control set_controller_state \
        joint_state_broadcaster active \
        --controller-manager "$CONTROLLER_MANAGER"
    timeout 10 ros2 control set_controller_state \
        "$ACTIVE_COMMAND_CONTROLLER" active \
        --controller-manager "$CONTROLLER_MANAGER"
fi

hardware_components="$(
    timeout 10 ros2 control list_hardware_components \
        --controller-manager "$CONTROLLER_MANAGER" 2>&1 | strip_ansi || true
)"
hardware_interfaces="$(
    timeout 10 ros2 control list_hardware_interfaces \
        --controller-manager "$CONTROLLER_MANAGER" 2>&1 | strip_ansi || true
)"
if ! grep -Eq 'label=active' <<<"$hardware_components" || \
    ! grep -Eq 'joint1/position \[available\] \[claimed\]' <<<"$hardware_interfaces" || \
    ! grep -Eq 'joint2/position \[available\] \[claimed\]' <<<"$hardware_interfaces" || \
    ! grep -Eq 'joint3/position \[available\] \[claimed\]' <<<"$hardware_interfaces" || \
    ! grep -Eq 'joint4/position \[available\] \[claimed\]' <<<"$hardware_interfaces" || \
    ! grep -Eq 'joint5/position \[available\] \[claimed\]' <<<"$hardware_interfaces" || \
    ! grep -Eq 'joint6/position \[available\] \[claimed\]' <<<"$hardware_interfaces"; then
    echo "$hardware_components" >&2
    echo "$hardware_interfaces" >&2
    echo "ERROR: HardwareInterface did not become active with joint1-6 claimed." >&2
    exit 1
fi
pass "HardwareInterface is active; joint1-6 position interfaces are available and claimed."

stage "Follower joint state from ros2_control"
check_topic_once /joint_states "$FOLLOWER_STATE_TIMEOUT"
pass "/joint_states is publishing follower joint1-6 feedback."

stage "Leader-to-follower relay"
if [[ "$RUN_MODE" == "gravity" ]]; then
    pass "Gravity-only mode: leader relay is not started."
else
    echo "Starting joint1-6 teleoperation relay."
    echo "Publishing: /joint1_teleop_controller/commands"

    setsid ros2 launch joint_state_relay \
        relay_to_microros_single_joint.launch.py &
    RELAY_PID=$!
    if ! check_topic_once \
        /joint1_teleop_controller/commands "$RELAY_START_TIMEOUT"; then
        if ! kill -0 "$RELAY_PID" 2>/dev/null; then
            wait "$RELAY_PID" 2>/dev/null || true
            echo "ERROR: joint_state_relay exited during startup." >&2
        else
            echo "ERROR: relay is running but did not publish a command." >&2
            echo "Check leader and follower timestamps and relative alignment logs." >&2
        fi
        exit 1
    fi
    echo "Relay output is healthy: /joint1_teleop_controller/commands"
    pass "Relay is publishing position commands for joint1-6."
fi

stage "Micro-ROS bridge command and acknowledgement"
check_topic_once /stm32/joint_commands "$RELAY_START_TIMEOUT"
pass "MicroRosSystem is publishing /stm32/joint_commands."

stm32_command_info="$(
    timeout 10 ros2 topic info /stm32/joint_commands 2>/dev/null || true
)"
stm32_subscription_count="$(
    awk -F': ' '/^Subscription count:/ {print $2; exit}' <<<"$stm32_command_info"
)"
if [[ ! "$stm32_subscription_count" =~ ^[1-9][0-9]*$ ]]; then
    echo "$stm32_command_info" >&2
    echo "ERROR: /stm32/joint_commands has no STM32/micro-ROS subscriber." >&2
    exit 1
fi
pass "/stm32/joint_commands has $stm32_subscription_count subscriber(s)."

ack_deadline=$((SECONDS + STM32_STATE_TIMEOUT))
last_command_sequence="0"
while (( SECONDS < ack_deadline )); do
    stm32_status_sample="$(
        timeout 5 ros2 topic echo /stm32/status --once 2>/dev/null || true
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

echo
if [[ "$RUN_MODE" == "gravity" ]]; then
    echo "Gravity-only self-check passed:"
    echo "  /stm32/joint_states -> gravity_compensation_controller"
    echo "  -> MicroRosSystem -> /stm32/joint_commands"
    echo "  -> micro-ROS Agent -> STM32 status acknowledgement"
    echo "Tune gravity_scales and effort_limits in single_joint_controllers.yaml."
    wait "$CONTROLLER_PID"
else
    echo "End-to-end self-check passed:"
    echo "  leader arm -> /leader/joint_states"
    echo "  -> joint_state_relay -> /joint1_teleop_controller/commands"
    echo "  -> ros2_control MicroRosSystem -> /stm32/joint_commands"
    echo "  -> micro-ROS Agent -> STM32 status acknowledgement"
    wait "$RELAY_PID"
fi
