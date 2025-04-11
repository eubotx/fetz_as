#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Float32, Bool
import math

class WeaponSpeedController(Node):
    def __init__(self):
        super().__init__('weapon_control')
        
        # Declare and get parameters
        self.declare_parameter('distance_threshold', 0.41)  # default 1.0 meter
        self.threshold = self.get_parameter('distance_threshold').value
        
        # Flags to track topic availability
        self.goal_pose_received = False
        self.robot_pose_received = False
        self.weapon_armed = False
        
        # Create publisher for weapon speed
        self.weapon_speed_pub = self.create_publisher(
            Float32,
            'weapon/speed',
            10
        )
        
        # Create subscribers
        self.goal_pose_sub = self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.goal_pose_callback,
            10
        )
        
        self.robot_pose_sub = self.create_subscription(
            PoseStamped,
            'camera/pose',
            self.robot_pose_callback,
            10
        )
        
        self.weapon_armed_sub = self.create_subscription(
            Bool,
            'weapon/armed',
            self.weapon_armed_callback,
            10
        )
        
        # Initialize variables
        self.current_goal_pose = None
        self.current_robot_pose = None
        
        # Create a timer for continuous publishing at 10Hz
        self.timer = self.create_timer(0.1, self.timer_callback)  # 0.1s = 10Hz
        
        self.get_logger().info("Weapon Speed Controller initialized")
        self.get_logger().info(f"Distance threshold: {self.threshold} meters")
        self.log_topic_status()
    
    def goal_pose_callback(self, msg):
        if not self.goal_pose_received:
            self.goal_pose_received = True
            self.get_logger().info("goal_pose topic is now available")
        self.current_goal_pose = msg.pose
    
    def robot_pose_callback(self, msg):
        if not self.robot_pose_received:
            self.robot_pose_received = True
            self.get_logger().info("camera/pose topic is now available")
        self.current_robot_pose = msg.pose
    
    def weapon_armed_callback(self, msg):
        self.weapon_armed = msg.data
        status = "ARMED" if self.weapon_armed else "DISARMED"
        self.get_logger().info(f"Weapon status changed: {status}")
    
    def log_topic_status(self):
        """Log the current status of required topics"""
        if not self.goal_pose_received:
            self.get_logger().warn("Waiting for goal_pose topic...")
        if not self.robot_pose_received:
            self.get_logger().warn("Waiting for camera/pose topic...")
        if self.goal_pose_received and self.robot_pose_received:
            self.get_logger().info("All required topics are available")
    
    def calculate_distance(self, pose1: Pose, pose2: Pose) -> float:
        """Calculate Euclidean distance between two poses"""
        if pose1 is None or pose2 is None:
            return float('inf')  # Return large distance if poses aren't available
        
        dx = pose1.position.x - pose2.position.x
        dy = pose1.position.y - pose2.position.y
        dz = pose1.position.z - pose2.position.z
        return math.sqrt(dx**2 + dy**2 + dz**2)
    
    def timer_callback(self):
        """Timer callback that publishes at 10Hz"""
        # Log topic status periodically (every 10 seconds)
        if self.get_clock().now().nanoseconds % 10_000_000_000 < 100_000_000:  # ~every 10 seconds
            self.log_topic_status()
        
        speed_msg = Float32()
        
        if not self.weapon_armed:
            speed_msg.data = 0.0
        else:
            distance = self.calculate_distance(self.current_goal_pose, self.current_robot_pose)
            if distance > self.threshold:
                speed_msg.data = 0.1
            else:
                speed_msg.data = 0.69
        
        self.weapon_speed_pub.publish(speed_msg)
        
        # Only log distance/speed when we have both poses and weapon is armed
        if (self.goal_pose_received and self.robot_pose_received and self.weapon_armed):
            self.get_logger().debug(f"Distance: {distance:.2f}m, Publishing speed: {speed_msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = WeaponSpeedController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()