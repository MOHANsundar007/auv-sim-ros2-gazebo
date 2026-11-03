import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'auv_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'models', 'auv'), glob('models/auv/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mohan',
    maintainer_email='gmohanasundaram17@gmail.com',
    description='AUV simulation on Gazebo Fortress + ROS 2 Humble',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'teleop = auv_sim.teleop:main',
            'autopilot = auv_sim.autopilot:main',
            'depth_sensor = auv_sim.depth_sensor:main',
        ],
    },
)
