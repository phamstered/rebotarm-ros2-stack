#!/bin/bash
# 本机: 同步代码到 Jetson 和 Ubuntu 并编译
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
SRC_DIR="$PROJECT_DIR/src"

SCRIPTS_DIR="$PROJECT_DIR/scripts"

echo "=== Syncing to Jetson ==="
rsync -avz --delete "$SRC_DIR/" damiao:~/Project/leader_follower/src/
rsync -avz --delete "$SCRIPTS_DIR/" damiao:~/Project/leader_follower/scripts/
ssh damiao "cd ~/Project/leader_follower && source /opt/ros/humble/setup.bash && colcon build 2>&1 | tail -10"

echo ""
echo "=== Syncing to Ubuntu ==="
rsync -avz --delete "$SRC_DIR/" ubuntu:~/Project/leader_follower/src/
rsync -avz --delete "$SCRIPTS_DIR/" ubuntu:~/Project/leader_follower/scripts/
ssh ubuntu "cd ~/Project/leader_follower && source /opt/ros/humble/setup.bash && colcon build 2>&1 | tail -10"

echo ""
echo "=== Done ==="
