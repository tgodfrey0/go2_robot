# BSD 3-Clause License

# Copyright (c) 2024, TODO
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
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import PushRosNamespace

ROS_NAMESPACE_PREFIX: str = "go2"  # Prefixed to the namespace, e.g. prefix_FF_FF_FF
ROS_NAMESPACE_SEPARATOR: str = (
    "_"  # Splits the prefix and the octets, e.g. for _ the ns is prefix_XX_XX_XX
)
ROS_NAMESPACE = f"{ROS_NAMESPACE_PREFIX}{ROS_NAMESPACE_SEPARATOR}{ROS_NAMESPACE_SEPARATOR.join((['{:02x}'.format((uuid.getnode() >> i) & 0xFF) for i in range(0, 48, 8)][::-1])[3:6])}"  # Each robot's namespace is the last 3 octets of its MAC address


def generate_launch_description():
    lidar = LaunchConfiguration("lidar")
    realsense = LaunchConfiguration("realsense")

    declare_lidar_cmd = DeclareLaunchArgument(
        "lidar", default_value="False", description="Launch hesai lidar driver"
    )

    declare_realsense_cmd = DeclareLaunchArgument(
        "realsense",
        default_value="False",
        description="Launch realsense driver",
    )

    robot_description_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("go2_description"), "launch/"),
                "robot.launch.py",
            ]
        )
    )

    driver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("go2_driver"), "launch/"),
                "go2_driver.launch.py",
            ]
        )
    )

    lidar_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("hesai_ros_driver"), "launch/"),
                "start.py",
            ]
        ),
        condition=IfCondition(PythonExpression([lidar])),
    )

    realsense_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("realsense2_camera"), "launch/"),
                "rs_launch.py",
            ]
        ),
        condition=IfCondition(PythonExpression([realsense])),
    )

    # Remap the actions into a namespace
    group = GroupAction(
        actions=[
            PushRosNamespace(ROS_NAMESPACE),
            robot_description_cmd,
            lidar_cmd,
            realsense_cmd,
            driver_cmd,
        ],
    )

    ld = LaunchDescription()

    # Add argument declarations
    ld.add_action(declare_lidar_cmd)
    ld.add_action(declare_realsense_cmd)

    # Add MRS group action
    ld.add_action(group)

    return ld
