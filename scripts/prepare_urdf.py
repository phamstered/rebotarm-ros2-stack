#!/usr/bin/env python3
"""Process xacro URDF and inject world fixed joint for Gazebo spawn."""
import xacro
import re
import os

ws = os.environ.get('GAZEBO_WS', os.path.expanduser('~/Project/leader_follower'))
pkg = ws + '/install/rebotarm_gazebo/share/rebotarm_gazebo'
xacro_path = pkg + '/urdf/rebotarm.urdf.xacro'

doc = xacro.parse(open(xacro_path))
xacro.process_doc(doc)
urdf = re.sub(r'<!--(.*?)-->', '', doc.toxml(), flags=re.DOTALL)
urdf = urdf.replace('package://rebotarm_gazebo/meshes/', pkg + '/meshes/')

# Inject world link + fixed joint
world_link = (
    '<link name="world">'
    '<inertial><mass value="0.0001"/>'
    '<inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>'
    '</inertial></link>'
)
world_joint = (
    '<joint name="world_fixed" type="fixed">'
    '<parent link="world"/>'
    '<child link="base_link"/>'
    '</joint>'
)
urdf = urdf.replace(
    '<robot name="rebotarm">',
    '<robot name="rebotarm">' + world_link + world_joint
)

output = '/tmp/rebotarm_description.urdf'
open(output, 'w').write(urdf)
print(f'URDF saved to {output} (with world fixed joint)')
