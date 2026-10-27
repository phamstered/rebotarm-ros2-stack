# ROS 2 Humble CI base image for rebotarm-ros2-stack.
#
# Bakes: apt deps from workspace package.xml trees (via rosdep) + pytest/yaml/ccache.
# Does NOT bake: full workspace source, colcon install space, hardware overlays,
#                stararm102_description, or micro_ros_agent.
#
# Image: ghcr.io/phamstered/rebotarm-ros2-ci
# Follow-up PR will pin .github/workflows/ci.yml to this image @sha256:<digest>.

FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=humble

# Copy only package.xml trees under src/ (including nested src/plugin/**)
# into a temp workspace for rosdep; removed before the image is finalized.
COPY src/joint_state_relay/package.xml \
     /tmp/ws/src/joint_state_relay/package.xml
COPY src/rebotarm_gazebo/package.xml \
     /tmp/ws/src/rebotarm_gazebo/package.xml
COPY src/rebotarm_hw_driver/package.xml \
     /tmp/ws/src/rebotarm_hw_driver/package.xml
COPY src/rebotarm_moveit_config/package.xml \
     /tmp/ws/src/rebotarm_moveit_config/package.xml
COPY src/plugin/microros_stm32h723_bridge/package.xml \
     /tmp/ws/src/plugin/microros_stm32h723_bridge/package.xml

RUN set -euo pipefail \
 && apt-get update \
 && apt-get install -y --no-install-recommends \
      python3-pytest \
      python3-yaml \
      ccache \
 && rosdep update \
 && rosdep install --from-paths /tmp/ws/src --ignore-src -y -r \
      --rosdistro "${ROS_DISTRO}" \
      --skip-keys "stararm102_description micro_ros_agent" \
 && rm -rf /var/lib/apt/lists/* /tmp/ws
