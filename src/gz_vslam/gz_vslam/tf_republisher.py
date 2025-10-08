#!/usr/bin/env python3
"""
TF Tree Fixer - Converts Gazebo flat TF to clean hierarchical tree
Removes Gazebo model prefixes (e.g., "x500_mono_cam_0/") and uses /clock for timestamps
Adds static camera frames to match camera_info frame_ids.
"""

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from rosgraph_msgs.msg import Clock
import copy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


class TFTreeFixer(Node):
    def __init__(self):
        super().__init__('tf_tree_fixer')

        # Latest transforms from Gazebo
        self.latest_transforms = {}

        # Latest simulation time
        self.sim_time = None

        # QoS profile for TF
        self.tf_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=100
        )

        # QoS for static TF
        self.static_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Prefix to strip (Gazebo model prefix)
        self.model_prefix = 'x500_mono_cam_0/'

        # Clean TF tree (child -> parent)
        self.tf_tree = {
            'base_link': 'odom',
            'body': 'base_link',
            'front_stereo_camera_left_optical': 'base_link',
            'front_stereo_camera_right_optical': 'base_link',
            'RSD455': 'body',
            'rotor0': 'body',
            'rotor1': 'body',
            'rotor2': 'body',
            'rotor3': 'body',
        }

        # Camera info frame mapping (static frames to match camera_info)
        self.camera_frames = {
            'x500_mono_cam_0/front_stereo_camera_left_optical/left_camera': 
                'x500_mono_cam_0/front_stereo_camera_left_optical',
            'x500_mono_cam_0/front_stereo_camera_right_optical/right_camera': 
                'x500_mono_cam_0/front_stereo_camera_right_optical'
        }

        # Subscribers
        self.tf_sub = self.create_subscription(
            TFMessage,
            '/tf_gazebo',
            self.tf_callback,
            self.tf_qos
        )

        self.clock_sub = self.create_subscription(
            Clock,
            '/clock',
            self.clock_callback,
            10
        )

        # Publishers
        self.tf_pub = self.create_publisher(TFMessage, '/tf', self.tf_qos)
        #self.tf_static_pub = self.create_publisher(TFMessage, '/tf_static', self.static_qos)

        self.timer = self.create_timer(0.02, self.publish_fixed_tf)
        #self.static_timer = self.create_timer(1.0, self.publish_static_camera_frames)

        self.get_logger().info('TF Tree Fixer started — using /clock and prefix stripping enabled.')

    def strip_prefix(self, frame_id: str) -> str:
        """Remove Gazebo model prefix if present"""
        if frame_id.startswith(self.model_prefix):
            return frame_id[len(self.model_prefix):]
        return frame_id

    def clock_callback(self, msg: Clock):
        """Store latest sim time"""
        self.sim_time = msg.clock

    def tf_callback(self, msg: TFMessage):
        """Store incoming transforms from Gazebo and strip prefix"""
        for transform in msg.transforms:
            # Strip prefix from both parent and child frames
            clean_child = self.strip_prefix(transform.child_frame_id)
            clean_parent = self.strip_prefix(transform.header.frame_id)
            
            # Store with cleaned child frame as key
            cleaned_transform = copy.deepcopy(transform)
            cleaned_transform.child_frame_id = clean_child
            cleaned_transform.header.frame_id = clean_parent
            
            self.latest_transforms[clean_child] = cleaned_transform

    def publish_fixed_tf(self):
        """Publish TF with clean hierarchy"""
        if not self.latest_transforms:
            return
        
        current_time = self.sim_time if self.sim_time else self.get_clock().now().to_msg()
        tf_msg = TFMessage()
        
        for child_frame, parent_frame in self.tf_tree.items():
            if child_frame in self.latest_transforms:
                transform = copy.deepcopy(self.latest_transforms[child_frame])
                transform.header.stamp = current_time
                transform.header.frame_id = parent_frame
                transform.child_frame_id = child_frame
                tf_msg.transforms.append(transform)
        
        if tf_msg.transforms:
            self.tf_pub.publish(tf_msg)

    # def publish_static_camera_frames(self):
    #     """Publish static frames for camera_info alignment"""
    #     current_time = self.sim_time if self.sim_time else self.get_clock().now().to_msg()
    #     static_tf_msg = TFMessage()
        
    #     for child_frame, parent_frame in self.camera_frames.items():
    #         t = TransformStamped()
    #         t.header.stamp = current_time
    #         t.header.frame_id = parent_frame
    #         t.child_frame_id = child_frame
    #         # Identity transform: camera frame coincides with optical link
    #         t.transform.translation.x = 0.0
    #         t.transform.translation.y = 0.0
    #         t.transform.translation.z = 0.0
    #         t.transform.rotation.x = 0.0
    #         t.transform.rotation.y = 0.0
    #         t.transform.rotation.z = 0.0
    #         t.transform.rotation.w = 1.0
    #         static_tf_msg.transforms.append(t)
        
    #     if static_tf_msg.transforms:
    #         self.tf_static_pub.publish(static_tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TFTreeFixer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
