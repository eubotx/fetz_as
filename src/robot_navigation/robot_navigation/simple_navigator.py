#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
import math

class SimpleNavigator(Node):
    def __init__(self):
        super().__init__('simple_navigator')
        
        # Parameters
        self.linear_kp = 0.5  # Proportional gain for linear velocity
        self.angular_kp = 1.0  # Proportional gain for angular velocity
        self.max_linear = 0.2  # m/s
        self.max_angular = 1.0  # rad/s
        self.position_tolerance = 0.05  # m
        self.orientation_tolerance = 0.1  # rad
        
        # Current state
        self.current_pose = None
        self.goal_pose = None
        
        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped, '/camera/robot_pose', self.pose_callback, 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)
            
        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Control timer
        self.timer = self.create_timer(0.033, self.control_loop)  # ~30Hz
    
    def pose_callback(self, msg):
        self.current_pose = msg.pose
    
    def goal_callback(self, msg):
        self.goal_pose = msg.pose
    
    def control_loop(self):
        if self.current_pose is None or self.goal_pose is None:
            return
            
        # Calculate errors
        dx = self.goal_pose.position.x - self.current_pose.position.x
        dy = self.goal_pose.position.y - self.current_pose.position.y
        distance = math.sqrt(dx**2 + dy**2)
        
        # Calculate desired yaw (you'll need to convert quaternions to Euler angles)
        desired_yaw = math.atan2(dy, dx)
        current_yaw = self.quaternion_to_yaw(self.current_pose.orientation)
        yaw_error = self.normalize_angle(desired_yaw - current_yaw)
        
        # Generate commands
        cmd = Twist()
        
        if distance > self.position_tolerance:
            # Point towards goal first
            if abs(yaw_error) > self.orientation_tolerance:
                cmd.angular.z = max(min(self.angular_kp * yaw_error, self.max_angular), -self.max_angular)
            else:
                cmd.linear.x = max(min(self.linear_kp * distance, self.max_linear), -self.max_linear)
        
        self.cmd_pub.publish(cmd)
    
    def quaternion_to_yaw(self, q):
        # Convert quaternion to yaw (simplified)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    navigator = SimpleNavigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()