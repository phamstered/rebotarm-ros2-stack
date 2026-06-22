from setuptools import setup, find_packages
from glob import glob

package_name = 'rebotarm_hw_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch/', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jialiang',
    maintainer_email='jialiang@todo.todo',
    description='reBotArm B601 hardware driver with ROS2 command interface',
    license='MIT',
    entry_points={
        'console_scripts': [
            'follower_driver = rebotarm_hw_driver.follower_driver_node:main',
        ],
    },
)
