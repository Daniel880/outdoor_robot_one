# Requirements:
#   A realsense D435i
#   Install realsense2 ros2 package (ros-$ROS_DISTRO-realsense2-camera)
# Example:
#   $ ros2 launch realsense2_camera rs_launch.py enable_gyro:=true enable_accel:=true unite_imu_method:=1 enable_sync:=true pointcloud.enable:=true
#
#   $ ros2 launch rtabmap_examples realsense_d435i_color.launch.py

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # ros2 launch realsense2_camera rs_launch.py enable_gyro:=true enable_accel:=true unite_imu_method:=1 enable_sync:=true pointcloud.enable:=true pointcloud.allow_no_texture_points:=true depth_module.profile:=1280x720x30


    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py'
                    )
            ]
        ),
        launch_arguments={
            "initial_reset": "true",
            "enable_gyro": "true",
            "enable_accel": "true",
            "unite_imu_method": "1",
            "enable_sync": "true",
            "pointcloud.enable": "true",
            "pointcloud.allow_no_texture_points": "true",
            "depth_module.profile": "1280x720x30",
        }.items(),
    )
    
    
    parameters=[{
          'frame_id':'base_link',
          'subscribe_depth':True,
          'subscribe_odom_info':True,
          'approx_sync':False,
          'wait_imu_to_init':True,
          'Grid/FromDepth':False,
          'Grid/MaxObstacleHeight':"1",
          'Reg/Force3DoF': "false"
          }]

    remappings=[
          ('imu', '/imu/data'),
          ('rgb/image', '/camera/color/image_raw'),
          ('rgb/camera_info', '/camera/color/camera_info'),
          ('depth/image', '/camera/depth/image_rect_raw'),
          ('odom', '/rtabmap_odom'),
          ]

    return LaunchDescription([

        # Nodes to launch       
        realsense_node,

        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=parameters,
            remappings=remappings),

        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d']),

        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=parameters,
            remappings=remappings),
                
        # Compute quaternion of the IMU
        Node(
            package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
            parameters=[{'use_mag': False, 
                         'world_frame':'enu', 
                         'publish_tf':False}],
            remappings=[('imu/data_raw', '/camera/imu')]),
        
        # The IMU frame is missing in TF tree, add it:
        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'camera_gyro_optical_frame', 'camera_imu_optical_frame']),
    ])
