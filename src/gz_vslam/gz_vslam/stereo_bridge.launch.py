#!/usr/bin/env python3
"""
Launch file to bridge Gazebo stereo camera, TF, and clock topics to ROS 2 topics.
Maps /stereo/* topics to /front_stereo_camera/* topics.
Also bridges /model/x500_mono_cam_0/pose and /world/default/clock.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with Gazebo–ROS2 bridges"""

    # Stereo camera bridges
    left_image_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='left_image_bridge',
        arguments=['/stereo/left/image_raw@sensor_msgs/msg/Image@gz.msgs.Image'],
        remappings=[('/stereo/left/image_raw', '/front_stereo_camera/left/image_rect_color')],
        output='screen'
    )

    right_image_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='right_image_bridge',
        arguments=['/stereo/right/image_raw@sensor_msgs/msg/Image@gz.msgs.Image'],
        remappings=[('/stereo/right/image_raw', '/front_stereo_camera/right/image_rect_color')],
        output='screen'
    )

    left_info_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='left_info_bridge',
        arguments=['/stereo/left/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'],
        remappings=[('/stereo/left/camera_info', '/front_stereo_camera/left/camera_info')],
        output='screen'
    )

    right_info_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='right_info_bridge',
        arguments=['/stereo/right/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'],
        remappings=[('/stereo/right/camera_info', '/front_stereo_camera/right/camera_info')],
        output='screen'
    )

    # TF bridge (pose from Gazebo model)
    tf_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='tf_bridge',
        arguments=['/model/x500_mono_cam_0/pose@geometry_msgs/msg/PoseStamped@gz.msgs.Pose'],
        remappings=[('/model/x500_mono_cam_0/pose', '/tf')],
        output='screen'
    )

    # Clock bridge
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/world/default/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'],
        remappings=[('/world/default/clock', '/clock')],
        output='screen'
    )

    return LaunchDescription([
        left_image_bridge,
        right_image_bridge,
        left_info_bridge,
        right_info_bridge,
        tf_bridge,
        clock_bridge,
    ])
