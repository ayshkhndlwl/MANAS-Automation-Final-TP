#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Pose2D
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data  # Required for Gazebo
import math

class DeadReckoningNode(Node):
    def __init__(self):
        super().__init__('dead_reckoning')
        
        print("=========== ODOMETRY RELAY BOOTING UP ===========", flush=True)
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pose_pub = self.create_publisher(Pose2D, '/estimated_pose', 10)
        
        # 1. Fixed the function name typo and added the Sensor QoS Profile
        self.odom_sub = self.create_subscription(
            Odometry, 
            '/odom', 
            self.odom_cb, 
            qos_profile_sensor_data)        
        
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def odom_cb(self, msg: Odometry):
        # 2. Extract coordinates
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # 3. Convert Quaternion to Theta
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.theta = math.atan2(siny_cosp, cosy_cosp)
        
# 4. Print the output (Throttled to twice a second)
        self.get_logger().info(f"[ODOMETRY ENGINE]: X={self.x:.3f}, Y={self.y:.3f}, Theta={self.theta:.3f}", throttle_duration_sec=0.5)        
        # 5. INSTANTLY PUBLISH! (This was missing)
        self.publish_data()

    def publish_data(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)
        self.tf_broadcaster.sendTransform(t)
        
        pose_msg = Pose2D()
        pose_msg.x = self.x
        pose_msg.y = self.y
        pose_msg.theta = self.theta
        self.pose_pub.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DeadReckoningNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()