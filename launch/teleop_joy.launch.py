# Copyright 2020 ros2_control Development Team
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


from launch import LaunchDescription
from launch.actions import GroupAction, DeclareLaunchArgument, RegisterEventHandler, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():


    joy_node = Node(
                package="joy",
                executable="joy_node",
            )
    
    teleop_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        parameters=[{
            'enable_button':5,
            'enable_turbo_button':4,
            'axis_linear.x':1,
            'axis_angular.yaw':3
            }],
        remappings=[("cmd_vel", "diff_drive_controller/cmd_vel_unstamped")],
    )

    nodes = [
        joy_node,
        teleop_node,
 
    ]
    return LaunchDescription(nodes)
