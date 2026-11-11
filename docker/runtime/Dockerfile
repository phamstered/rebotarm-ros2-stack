FROM ros:humble-ros-base-jammy
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /opt/rebotarm
COPY src/ src/
COPY scripts/ scripts/
# External leader description and micro-ROS agent are supplied as overlays.
RUN apt-get update \
 && apt-get install -y --no-install-recommends python3-colcon-common-extensions python3-rosdep python3-pytest python3-yaml procps iproute2 \
 && rosdep update --rosdistro humble \
 && rosdep install --from-paths src --ignore-src -y --rosdistro humble --skip-keys 'stararm102_description micro_ros_agent' \
 && source /opt/ros/humble/setup.bash \
 && colcon build --executor sequential --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release \
 && rm -rf /var/lib/apt/lists/*
COPY docker/runtime/entrypoint.sh /usr/local/bin/rebotarm-entrypoint
ENV LEADER_FOLLOWER_WS=/opt/rebotarm
ENTRYPOINT ["/usr/local/bin/rebotarm-entrypoint"]
CMD ["bash"]
