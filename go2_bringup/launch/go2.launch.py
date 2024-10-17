# BSD 3-Clause License

# Copyright (c) 2024, Intelligent Robotics Lab
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.

# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.

# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import os
import uuid

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace

ns = '_'.join((['{:02x}'.format((uuid.getnode() >> i) & 0xff) for i in range(0,48,8)][::-1])[3:6]) # Each robot's namespace is the last 3 octets of its MAC address

def generate_launch_description():
    lidar = LaunchConfiguration('lidar')
    realsense = LaunchConfiguration('realsense')
    rviz = LaunchConfiguration('rviz')
    mrs = LaunchConfiguration('mrs')

    declare_lidar_cmd = DeclareLaunchArgument(
        'lidar',
        default_value='False',
        description='Launch hesai lidar driver'
    )

    declare_realsense_cmd = DeclareLaunchArgument(
        'realsense',
        default_value='False',
        description='Launch realsense driver'
    )

    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz',
        default_value='False',
        description='Launch rviz'
    )
    
    declare_mrs_cmd = DeclareLaunchArgument(
        'mrs',
        default_value='False',
        description='Launch with remapping for multi-robot system usage'
    )

    robot_description_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('go2_description'),
            'launch/'), 'robot.launch.py'])
    )

    driver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('go2_driver'),
            'launch/'), 'go2_driver.launch.py'])
    )

    lidar_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('hesai_ros_driver'),
            'launch/'), 'start.py']),
        condition=IfCondition(PythonExpression([lidar]))
    )

    realsense_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('realsense2_camera'),
            'launch/'), 'rs_launch.py']),
        condition=IfCondition(PythonExpression([realsense]))
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('go2_rviz'),
            'launch/'), 'rviz.launch.py']),
        condition=IfCondition(PythonExpression([rviz]))
    )
    
    # Remap the actions into a namespace if `mrs` is true
    mrs_group = GroupAction(
        actions=[
            PushRosNamespace(ns),
            robot_description_cmd,
            lidar_cmd,
            realsense_cmd,
            driver_cmd,
            rviz_cmd
        ],
        condition=IfCondition(mrs)
    )

    # Actions for non-MRS mode
    non_mrs_actions = [
        robot_description_cmd,
        lidar_cmd,
        realsense_cmd,
        driver_cmd,
        rviz_cmd
    ]

    ld = LaunchDescription()
    
    # Add argument declarations
    ld.add_action(declare_lidar_cmd)
    ld.add_action(declare_realsense_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_mrs_cmd)

    # Add MRS group action
    ld.add_action(mrs_group)

    # Add non-MRS actions with conditions
    for action in non_mrs_actions:
        ld.add_action(GroupAction(
            [action],
            condition=IfCondition(PythonExpression(['not ', mrs]))
        ))

    return ld
