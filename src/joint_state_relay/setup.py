from setuptools import setup, find_packages
from glob import glob

package_name = 'joint_state_relay'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch/', glob('launch/*.py')),
        ('share/' + package_name + '/config/', glob('config/*.yaml') + glob('config/*.xml')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='jialiang',
    maintainer_email='jialiang@todo.todo',
    description='Joint state relay between leader and follower robotic arms',
    license='MIT',
    entry_points={
        'console_scripts': [
            'relay_node = joint_state_relay.relay_node:main',
            'leader_driver = joint_state_relay.leader_driver:main',
            'domain_bridge = joint_state_relay.domain_bridge:main',
            'calibrate_gripper = joint_state_relay.gripper_calibration:main',
        ],
    },
)
