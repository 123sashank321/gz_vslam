import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
import numpy as np

class OdometryToPx4Bridge(Node):
    def __init__(self):
        super().__init__('odometry_to_px4_bridge')
        self.sub = self.create_subscription(
            Odometry,
            '/visual_slam/tracking/odometry',
            self.odom_callback,
            10
        )
        self.pub = self.create_publisher(
            VehicleOdometry,
            '/fmu/in/vehicle_visual_odometry',
            10
        )

    def odom_callback(self, msg):
        self.get_logger().info('Received Odometry message')
        odom = VehicleOdometry()
        odom.timestamp = int(self.get_clock().now().nanoseconds / 1000)  # microseconds
        odom.pose_frame = VehicleOdometry.POSE_FRAME_NED  # or FRD as needed

        # Position
        odom.position = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ]
        # Orientation (w, x, y, z)
        odom.q = [
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z
        ]
        # Linear velocity
        odom.velocity = [
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ]
        # Angular velocity
        odom.angular_velocity = [
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z
        ]
        # Covariances
        odom.position_variance = [
            msg.pose.covariance[0],  # x
            msg.pose.covariance[7],  # y
            msg.pose.covariance[14]  # z
        ]
        odom.orientation_variance = [
            msg.pose.covariance[21], # roll
            msg.pose.covariance[28], # pitch
            msg.pose.covariance[35]  # yaw
        ]
        odom.velocity_variance = [
            msg.twist.covariance[0],  # vx
            msg.twist.covariance[7],  # vy
            msg.twist.covariance[14]  # vz
        ]
        self.pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryToPx4Bridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()