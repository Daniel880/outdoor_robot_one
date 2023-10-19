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

import os

from ament_index_python.packages import get_package_share_directory

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
    world_path = PathJoinSubstitution(
        [
            FindPackageShare("outdoor_robot_one"),
            "worlds",
            'mcmillan_airfield.world',
        ]
    )
    
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "world",
            default_value=world_path,
            description="Start RViz2 automatically with this launch file.",
        )
    )
    
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "use_mock_hardware",
    #         default_value="false",
    #         description="Start robot with mock hardware mirroring command to its states.",
    #     )
    # )



    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("outdoor_robot_one"), "description", "robot.urdf.xacro"]
            ),
            " ",
            "sim_mode:=true",
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("outdoor_robot_one"),
            "configuration",
            "ros2_controllers.yaml",
        ]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("outdoor_robot_one"), "rviz", "robot.rviz"]
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        remappings=[
            ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
        ],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
    )

    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
            )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'robot'],
                        output='screen')
    

    rl_params_file = os.path.join(get_package_share_directory("outdoor_robot_one"), 'configuration', 'dual_ekf_navsat_params.yaml')

    navsat_transform_node = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform",
        output="screen",
        parameters=[rl_params_file],
        remappings=[
            ("imu", "mavros/imu/data"),
            ("gps/fix", "gps/fix"),
            ("gps/filtered", "gps/filtered"),
            ("odometry/gps", "odometry/gps"),
            ("odometry/filtered", "odometry/global"),
        ],
    )

    ekf_filter_node_odom = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_odom",
        output="screen",
        parameters=[rl_params_file],
        remappings=[("odometry/filtered", "odometry/local")],
    )

    ekf_filter_node_map = Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_map",
                output="screen",
                parameters=[rl_params_file],
                remappings=[("odometry/filtered", "odometry/global")],
            )
    

    nav2_params = os.path.join(get_package_share_directory("outdoor_robot_one"), 'configuration', 'nav2_no_map_params.yaml')
    
    nav_include = GroupAction(
        actions=[

            SetRemap(src='/cmd_vel',dst='/diff_drive_controller/cmd_vel_unstamped'),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(                
                    os.path.join(get_package_share_directory('nav2_bringup'), "launch", "navigation_launch.py")
                    ),
            launch_arguments={
                "use_sim_time": "false",
                "params_file": nav2_params,
                "autostart": "True",
            }.items(),

            )
        ]
    )

    apm_node = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(     [        
            os.path.join(get_package_share_directory('outdoor_robot_one'), "launch", "apm.launch")
            ])
        )


    nodes = [
        gazebo,
        spawn_entity,
        robot_state_pub_node,
        # controller_manager_node,
        rviz_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        navsat_transform_node,
        ekf_filter_node_odom,
        ekf_filter_node_map,
        nav_include, 
        # apm_node
    ]
    return LaunchDescription(declared_arguments + nodes)
