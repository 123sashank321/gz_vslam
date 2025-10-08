#!/usr/bin/env python3
"""
Complete launch file for stereo camera with TF transforms
Bridges Gazebo poses to ROS 2 TF tree
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with TF and topic bridges"""
    
    # TF Bridge - converts Gazebo pose messages to TF transforms
    # This publishes: odom->base_link, base_link->body, base_link->optical_frames, body->rotors
    
    odom_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='odom_bridge',
        arguments=['/model/x500_mono_cam/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'],
        remappings=[('/model/x500_mono_cam/odometry', '/body/odom')],
        output='screen'
    )
    
    tf_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='tf_bridge',
        arguments=['/model/x500_mono_cam_0/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
        remappings=[('/model/x500_mono_cam_0/pose', '/tf_gazebo')],
        output='screen'
    )
    
    left_image_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='left_image_bridge',
        arguments=['left/image_raw@sensor_msgs/msg/Image@gz.msgs.Image'],
        #remappings=[('front_stereo_camera/left/image_raw', '/front_stereo_camera/left/image_rect_color')],
        output='screen'
    )
    
    # Bridge right camera image
    right_image_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='right_image_bridge',
        arguments=['right/image_raw@sensor_msgs/msg/Image@gz.msgs.Image'],
        #remappings=[('front_stereo_camera/right/image_raw','/front_stereo_camera/right/image_rect_color')],
        output='screen'
    )
    
    # Bridge left camera info
    left_info_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='left_info_bridge',
        arguments=['left/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'],
        #remappings=[('front_stereo_camera/left/camera_info','/front_stereo_camera/left/camera_info')],
        output='screen'
    )
    
    # Bridge right camera info
    right_info_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='right_info_bridge',
        arguments=['right/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'],
        #remappings=[('front_stereo_camera/right/camera_info','/front_stereo_camera/right/camera_info')],
        output='screen'
    )
    # Bridge clock
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )
    
    tf_republisher_node = Node(
        package='gz_vslam',
        executable='tf_republisher',
        name='tf_tree_fixer',
        output='screen',
        parameters=[{'use_sim_time': True}]  # make sure it uses /clock
    )
    
    camera_republisher_node = Node(
        package='gz_vslam',
        executable='camera_remapper',
        name='camera_remapper',
        output='screen',
        parameters=[{'use_sim_time': True}]  # make sure it uses /clock
    )
    
    return LaunchDescription([
        odom_bridge,
        tf_bridge,
        left_image_bridge,
        right_image_bridge,
        left_info_bridge,
        right_info_bridge,
        clock_bridge,
        tf_republisher_node,
        camera_republisher_node
    ])