#!/usr/bin/env python3
"""
Camera Info and Image Frame ID Fixer
Strips Gazebo model prefix from camera_info and image frame_id
Republishes with clean frame names matching TF tree
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


class CameraFixer(Node):
    def __init__(self):
        super().__init__('camera_fixer')
        
        # Declare parameters
        #self.declare_parameter('use_sim_time', True)
        
        # Prefix to strip from frame_id
        self.model_prefix = 'x500_mono_cam_0/'
        
        # QoS profiles
        self.camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        
        # ========== LEFT CAMERA ==========
        # Subscribe to raw topics from Gazebo bridge
        self.left_image_sub = self.create_subscription(
            Image,
            '/left/image_raw',
            self.left_image_callback,
            self.camera_qos
        )
        
        self.left_info_sub = self.create_subscription(
            CameraInfo,
            '/left/camera_info',
            self.left_info_callback,
            self.camera_qos
        )
        
        # Publish to clean topics for visual SLAM
        self.left_image_pub = self.create_publisher(
            Image,
            '/front_stereo_camera/left/image_rect_color',
            self.camera_qos
        )
        
        self.left_info_pub = self.create_publisher(
            CameraInfo,
            '/front_stereo_camera/left/camera_info',
            self.camera_qos
        )
        
        # ========== RIGHT CAMERA ==========
        # Subscribe to raw topics from Gazebo bridge
        self.right_image_sub = self.create_subscription(
            Image,
            '/right/image_raw',
            self.right_image_callback,
            self.camera_qos
        )
        
        self.right_info_sub = self.create_subscription(
            CameraInfo,
            '/right/camera_info',
            self.right_info_callback,
            self.camera_qos
        )
        
        # Publish to clean topics for visual SLAM
        self.right_image_pub = self.create_publisher(
            Image,
            '/front_stereo_camera/right/image_rect_color',
            self.camera_qos
        )
        
        self.right_info_pub = self.create_publisher(
            CameraInfo,
            '/front_stereo_camera/right/camera_info',
            self.camera_qos
        )
        
        self.get_logger().info('Camera Fixer started - processing left and right cameras')
        # self.get_logger().info('  Input: /left/image_raw, /left/camera_info')
        # self.get_logger().info('  Input: /right/image_raw, /right/camera_info')
        # self.get_logger().info('  Output: /front_stereo_camera/left/image_rect_color, /front_stereo_camera/left/camera_info')
        # self.get_logger().info('  Output: /front_stereo_camera/right/image_rect_color, /front_stereo_camera/right/camera_info')
    
    def strip_prefix(self, frame_id: str) -> str:
        """Remove Gazebo model prefix and sensor suffix, return clean frame name"""
        # Remove model prefix (e.g., "x500_mono_cam_0/")
        if frame_id.startswith(self.model_prefix):
            frame_id = frame_id[len(self.model_prefix):]
        
        # Remove sensor suffix (e.g., "/left_camera" or "/right_camera")
        if frame_id.endswith('/left_camera'):
            frame_id = frame_id[:-len('/left_camera')]
        elif frame_id.endswith('/right_camera'):
            frame_id = frame_id[:-len('/right_camera')]
        
        return frame_id
    
    # ========== LEFT CAMERA CALLBACKS ==========
    def left_image_callback(self, msg: Image):
        """Process and republish left camera image with clean frame_id"""
        fixed_msg = Image()
        fixed_msg.header = msg.header
        fixed_msg.header.frame_id = self.strip_prefix(msg.header.frame_id)
        fixed_msg.height = msg.height
        fixed_msg.width = msg.width
        fixed_msg.encoding = msg.encoding
        fixed_msg.is_bigendian = msg.is_bigendian
        fixed_msg.step = msg.step
        fixed_msg.data = msg.data
        
        self.left_image_pub.publish(fixed_msg)
    
    def left_info_callback(self, msg: CameraInfo):
        """Process and republish left camera info with clean frame_id"""
        fixed_msg = CameraInfo()
        fixed_msg.header = msg.header
        fixed_msg.header.frame_id = self.strip_prefix(msg.header.frame_id)
        fixed_msg.height = msg.height
        fixed_msg.width = msg.width
        fixed_msg.distortion_model = msg.distortion_model
        fixed_msg.d = msg.d
        fixed_msg.k = msg.k
        fixed_msg.r = msg.r
        fixed_msg.binning_x = msg.binning_x
        fixed_msg.binning_y = msg.binning_y
        fixed_msg.roi = msg.roi

        # ===== Override the projection matrix manually =====
        fx = 381.3611
        fy = 381.3611
        cx = 320.0
        cy = 180.0
        # Left camera (no baseline offset)
        fixed_msg.p = [
            fx, 0.0, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]
        self.left_info_pub.publish(fixed_msg)
    
    def left_info_callback_1(self, msg: CameraInfo):
        """Process and republish left camera info with clean frame_id"""
        fixed_msg = CameraInfo()
        fixed_msg.header = msg.header
        fixed_msg.header.frame_id = self.strip_prefix(msg.header.frame_id)
        fixed_msg.height = msg.height
        fixed_msg.width = msg.width
        fixed_msg.distortion_model = msg.distortion_model
        fixed_msg.d = msg.d
        fixed_msg.k = msg.k
        fixed_msg.r = msg.r
        fixed_msg.p = msg.p
        fixed_msg.binning_x = msg.binning_x
        fixed_msg.binning_y = msg.binning_y
        fixed_msg.roi = msg.roi
        
        self.left_info_pub.publish(fixed_msg)
    
    # ========== RIGHT CAMERA CALLBACKS ==========
    def right_image_callback(self, msg: Image):
        """Process and republish right camera image with clean frame_id"""
        fixed_msg = Image()
        fixed_msg.header = msg.header
        fixed_msg.header.frame_id = self.strip_prefix(msg.header.frame_id)
        fixed_msg.height = msg.height
        fixed_msg.width = msg.width
        fixed_msg.encoding = msg.encoding
        fixed_msg.is_bigendian = msg.is_bigendian
        fixed_msg.step = msg.step
        fixed_msg.data = msg.data
        
        self.right_image_pub.publish(fixed_msg)
    
    def right_info_callback(self, msg: CameraInfo):
        """Process and republish right camera info with clean frame_id"""
        fixed_msg = CameraInfo()
        fixed_msg.header = msg.header
        fixed_msg.header.frame_id = self.strip_prefix(msg.header.frame_id)
        fixed_msg.height = msg.height
        fixed_msg.width = msg.width
        fixed_msg.distortion_model = msg.distortion_model
        fixed_msg.d = msg.d
        fixed_msg.k = msg.k
        fixed_msg.r = msg.r
        fixed_msg.binning_x = msg.binning_x
        fixed_msg.binning_y = msg.binning_y
        fixed_msg.roi = msg.roi

        # ===== Override the projection matrix manually =====
        fx = 381.3611
        fy = 381.3611
        cx = 320.0
        cy = 180.0
        baseline = 0.09  # meters
        # Right camera (apply baseline offset)
        fixed_msg.p = [
            fx, 0.0, cx, -fx * baseline,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]

        self.right_info_pub.publish(fixed_msg)

    
    def right_info_callback_1(self, msg: CameraInfo):
        """Process and republish right camera info with clean frame_id"""
        fixed_msg = CameraInfo()
        fixed_msg.header = msg.header
        fixed_msg.header.frame_id = self.strip_prefix(msg.header.frame_id)
        fixed_msg.height = msg.height
        fixed_msg.width = msg.width
        fixed_msg.distortion_model = msg.distortion_model
        fixed_msg.d = msg.d
        fixed_msg.k = msg.k
        fixed_msg.r = msg.r
        fixed_msg.p = msg.p
        fixed_msg.binning_x = msg.binning_x
        fixed_msg.binning_y = msg.binning_y
        fixed_msg.roi = msg.roi
        
        self.right_info_pub.publish(fixed_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraFixer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()