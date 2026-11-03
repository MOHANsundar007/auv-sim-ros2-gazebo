"""Launch Gazebo Fortress with the ocean world, spawn the AUV, and bridge to ROS 2.

  ros2 launch auv_sim sim.launch.py                 # Gazebo GUI
  ros2 launch auv_sim sim.launch.py rviz:=true      # + RViz
  ros2 launch auv_sim sim.launch.py gui:=false      # headless
  ros2 launch auv_sim sim.launch.py z:=-10 yaw:=1.57
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('auv_sim')
    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')

    args = [
        DeclareLaunchArgument('world', default_value=os.path.join(pkg, 'worlds', 'ocean.sdf')),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='-3.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
    ]

    # Let Gazebo resolve model://auv
    resource_path = AppendEnvironmentVariable(
        'IGN_GAZEBO_RESOURCE_PATH', os.path.join(pkg, 'models'))

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            # -r: start running; -s: server only when gui:=false
            'gz_args': [PythonExpression(["'-r ' + ('' if '", gui, "' == 'true' else '-s ')"]),
                        world],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-world', 'ocean', '-name', 'auv',
                   '-file', os.path.join(pkg, 'models', 'auv', 'model.sdf'),
                   '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                   '-z', LaunchConfiguration('z'), '-Y', LaunchConfiguration('yaw')],
    )

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        parameters=[{'config_file': os.path.join(pkg, 'config', 'bridge.yaml'),
                     'use_sim_time': True}],
    )

    camera_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['--x', '0.88', '--frame-id', 'base_link', '--child-frame-id', 'camera_link'],
        parameters=[{'use_sim_time': True}],
    )

    depth_sensor = Node(
        package='auv_sim', executable='depth_sensor',
        parameters=[{'use_sim_time': True}],
    )

    rviz = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', os.path.join(pkg, 'rviz', 'auv.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription(args + [resource_path, gazebo, spawn, bridge,
                                     camera_tf, depth_sensor, rviz])
