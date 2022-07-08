#!/usr/bin/env python

# Copyright 1996-2021 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch Webots and the controller."""

import os
import pathlib
import launch
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from webots_ros2_driver.webots_launcher import WebotsLauncher

def generate_launch_description():

    optional_nodes = []

    package_dir = get_package_share_directory('tiago_assignment')

    local_params_file = LaunchConfiguration('local_params')
    lifelong_params_file = LaunchConfiguration('lifelong_params')
    slam_params_file = LaunchConfiguration('slam_params')
    nav_params = LaunchConfiguration('nav_params')

    the_map = LaunchConfiguration('map')
    world = LaunchConfiguration('world')
    mode = LaunchConfiguration('mode')

    slam_util = LaunchConfiguration('slam', default=False)
    localization_util = LaunchConfiguration('localization', default=False)
    rviz_util = LaunchConfiguration('rviz', default=False)
    nav_util = LaunchConfiguration('nav', default=False)
    lifelong_util = LaunchConfiguration('lifelong', default=False)

    robot_description = pathlib.Path(os.path.join(package_dir, 'resource', 'tiago_webots.urdf')).read_text()
    ros2_params_control = os.path.join(package_dir, 'resource', 'ros2_control.yml')
    navigation2_map = PathJoinSubstitution([package_dir, 'resource', the_map])

    local_params = PathJoinSubstitution([package_dir, 'configurations', local_params_file])
    navigation2_params = PathJoinSubstitution([package_dir, 'configurations', nav_params])
    slam_params = PathJoinSubstitution([package_dir, 'configurations', slam_params_file])
    lifelong_params = PathJoinSubstitution([package_dir, 'configurations', lifelong_params_file])
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'environments', world]),
        mode=mode
    )

    controller_manager_timeout = ['--controller-manager-timeout', '50']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''

    use_deprecated_spawner_py = 'ROS_DISTRO' in os.environ and os.environ['ROS_DISTRO'] == 'foxy'

    diffdrive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner' if not use_deprecated_spawner_py else 'spawner.py',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['diffdrive_controller'] + controller_manager_timeout,
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner' if not use_deprecated_spawner_py else 'spawner.py',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['joint_state_broadcaster'] + controller_manager_timeout,
    )

    tiago_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        parameters=[
            {'robot_description': robot_description,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
            ros2_params_control
        ],
        remappings=[
            ('/diffdrive_controller/cmd_vel_unstamped', '/cmd_vel')
        ]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>'
        }],
    )

    footprint_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
    )

    rviz_config = os.path.join(get_package_share_directory('tiago_assignment'), 'resource', 'default.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['--display-config=' + rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=launch.conditions.IfCondition(rviz_util)
    )

    if 'slam_toolbox' in get_packages_with_prefixes():
        optional_nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
            launch_arguments=[
                ('use_sim_time', use_sim_time),
                ('slam_params_file',slam_params),
            ],
            condition=launch.conditions.IfCondition(slam_util)))
            
    if 'slam_toolbox' in get_packages_with_prefixes():
        optional_nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
            launch_arguments=[
                ('use_sim_time', use_sim_time),
                ('slam_params_file',local_params),
            ],
            condition=launch.conditions.IfCondition(localization_util)))

    if 'slam_toolbox' in get_packages_with_prefixes():
        optional_nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('slam_toolbox'), 'launch', 'lifelong_launch.py')),
            launch_arguments=[
                ('use_sim_time', use_sim_time),
                ('slam_params_file',lifelong_params),
            ],
            condition=launch.conditions.IfCondition(lifelong_util)))

    if 'nav2_bringup' in get_packages_with_prefixes():
        optional_nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
            launch_arguments=[
                ('map', navigation2_map),
                ('use_sim_time', use_sim_time),
                ('params_file',navigation2_params),
            ],
            condition=launch.conditions.IfCondition(nav_util)))
            
    if 'nav2_bringup' in get_packages_with_prefixes():
        optional_nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
            launch_arguments=[
                ('map', navigation2_map),
                ('use_sim_time', use_sim_time),
                ('params_file',navigation2_params),
            ],
            condition=launch.conditions.IfCondition(localization_util)))
            
    if 'nav2_bringup' in get_packages_with_prefixes():
        optional_nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
            launch_arguments=[
                ('map', navigation2_map),
                ('use_sim_time', use_sim_time),
                ('params_file',navigation2_params),
            ],
            condition=launch.conditions.IfCondition(slam_util)))
    
    if 'nav2_bringup' in get_packages_with_prefixes():
        optional_nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
            launch_arguments=[
                ('map', navigation2_map),
                ('use_sim_time', use_sim_time),
                ('params_file',navigation2_params),
            ],
            condition=launch.conditions.IfCondition(lifelong_util)))

    return LaunchDescription([
        DeclareLaunchArgument(
            # In order to have another world laoded, instead of break_room_1, insert break_room_2.wbt/warehouse.wbt/house.wbt 
            'world',
            default_value='break_room_1.wbt',
            description='Select one world files from `/tiago_assignment/environments` directory'
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='realtime',
            description='Webots startup mode'
        ),
        DeclareLaunchArgument(
            'map',
            default_value='map.yaml',
            description='Select one map files from `/tiago_assignment/resource` directory'
        ),
        DeclareLaunchArgument(
            'nav_params',
            default_value='navigation2_parameters.yaml',
            description='Select one parameters files from `/tiago_assignment/configurations` directory'
        ),
        DeclareLaunchArgument(
            'slam_params',
            default_value='slam_parameters.yaml',
            description='Select one parameters files from `/tiago_assignment/configurations` directory'
        ),
        DeclareLaunchArgument(
            'local_params',
            default_value='localization_parameters.yaml',
            description='Select one parameters files from `/tiago_assignment/configurations` directory'
        ),
        DeclareLaunchArgument(
            'lifelong_params',
            default_value = 'lifelong_parameters.yaml',
            description= 'Select one parameters files from `tiago_assignment/configurations` directory'
        ),
        
        joint_state_broadcaster_spawner,
        diffdrive_controller_spawner,
        webots,
        rviz,
        robot_state_publisher,
        tiago_driver,
        footprint_publisher,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ] + optional_nodes)