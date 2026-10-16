#!/bin/bash
set -e

# ── Environment (works via SSH non-login shell) ──
source /opt/ros/humble/setup.bash
source /home/jialiang/Project/rebotarm-ros2-stack/install/setup.bash
export LD_LIBRARY_PATH=/opt/ros/humble/lib:$LD_LIBRARY_PATH
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=/opt/ros/humble/lib
export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/humble/lib
export IGN_GAZEBO_RESOURCE_PATH=/home/jialiang/Project/rebotarm-ros2-stack/install/rebotarm_gazebo/share/rebotarm_gazebo/models
export DISPLAY=:99
export ROS_DOMAIN_ID=1

# ── Process xacro → URDF ──
python3 -c "
import xacro, re
doc = xacro.parse(open('/home/jialiang/Project/rebotarm-ros2-stack/install/rebotarm_gazebo/share/rebotarm_gazebo/urdf/rebotarm.urdf.xacro'))
xacro.process_doc(doc)
text = re.sub(r'<!--(.*?)-->', '', doc.toxml(), flags=re.DOTALL)
text = text.replace('package://rebotarm_gazebo/meshes/', '/home/jialiang/Project/rebotarm-ros2-stack/install/rebotarm_gazebo/share/rebotarm_gazebo/meshes/')
open('/tmp/rebotarm_description.urdf', 'w').write(text)
print('URDF saved')
"

# ── Start Ignition Gazebo 6 (server-only) ──
ign gazebo -r -v 4 -s empty.sdf &
GZ_PID=$!
sleep 8

# ── Start robot_state_publisher ──
ros2 run robot_state_publisher robot_state_publisher --ros-args -p use_sim_time:=true -p "robot_description:=$(cat /tmp/rebotarm_description.urdf)" &
sleep 2

# ── Spawn model ──
SDF=$(cat /tmp/rebotarm_description.urdf)
ign service -s /world/empty/create \
  --reqtype ignition.msgs.EntityFactory \
  --reptype ignition.msgs.Boolean \
  --timeout 5000 \
  -r "sdf: '$SDF' name: 'rebotarm' allow_renaming: true"
sleep 3

# ── Load ros2 controllers ──
ros2 run controller_manager spawner joint_state_broadcaster &
ros2 run controller_manager spawner arm_controller &

echo "=== Ignition Gazebo 6 simulation running ==="
wait $GZ_PID
