#!/usr/bin/env python3

# Copyright (c) 2023 Franka Robotics GmbH
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

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='172.16.0.2',
        description='IP address of the Franka robot'
    )
    
    arm_id_arg = DeclareLaunchArgument(
        'arm_id',
        default_value='panda',
        description='Name of the robot arm'
    )

    # Load controller using ros2 control commands
    load_controller = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'load_controller',
            '--set-state', 'inactive',
            'pick_and_place_controller'
        ],
        output='screen'
    )

    # Set controller parameters
    set_controller_type = ExecuteProcess(
        cmd=[
            'ros2', 'param', 'set',
            '/controller_manager',
            'pick_and_place_controller.type',
            'franka_example_controllers/PickAndPlaceController'
        ],
        output='screen'
    )

    # Activate controller after loading
    activate_controller = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'control', 'set_controller_state',
                    'pick_and_place_controller', 'active'
                ],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        robot_ip_arg,
        arm_id_arg,
        LogInfo(msg="Loading Pick and Place Controller"),
        set_controller_type,
        load_controller,
        activate_controller
    ])