#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
import math
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class SimpleNavigator(Node):
    def __init__(self):
        super().__init__('simple_navigator')
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('linear_kp', 0.5),
                ('angular_kp', 1.0),
                ('max_linear', 0.2),
                ('max_angular', 1.0),
                ('position_tolerance', 0.05),
                ('orientation_tolerance', 0.1),
                ('goal_frame', 'map')
            ]
        )
        
        # Get parameters
        self.linear_kp = self.get_parameter('linear_kp').value
        self.angular_kp = self.get_parameter('angular_kp').value
        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value
        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.orientation_tolerance = self.get_parameter('orientation_tolerance').value
        self.goal_frame = self.get_parameter('goal_frame').value
        
        # Current state
        self.current_pose = None
        self.current_pose_header = None
        self.goal_pose = None
        self.goal_pose_header = None
        
        # TF2 listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped, 
            '/camera/pose', 
            self.pose_callback, 
            10)
            
        self.goal_sub = self.create_subscription(
            PoseStamped, 
            'camera/enemy/pose', 
            self.goal_callback, 
            10)
            
        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Control timer (30Hz to match your camera)
        self.control_timer = self.create_timer(1.0/30.0, self.control_loop)
        
        self.get_logger().info("Simple Navigator initialized")

    def pose_callback(self, msg):
        """Store the current robot pose from camera"""
        self.current_pose = msg.pose  # Changed from msg.pose.pose to msg.pose
        self.current_pose_header = msg.header
        
    def goal_callback(self, msg):
        """Store the goal pose"""
        self.goal_pose = msg.pose
        self.goal_pose_header = msg.header
        self.get_logger().info(f"New goal received: {self.goal_pose.position}")

    def control_loop(self):
        if self.current_pose is None or self.current_pose_header is None:
            self.get_logger().warn("Waiting for current pose...", throttle_duration_sec=5.0)
            return
            
        if self.goal_pose is None or self.goal_pose_header is None:
            self.get_logger().warn("Waiting for goal pose...", throttle_duration_sec=5.0)
            return
            
        try:
            # Get current position in goal frame
            transform = self.tf_buffer.lookup_transform(
                self.goal_pose_header.frame_id,  # Changed to use goal's frame
                self.current_pose_header.frame_id,
                rclpy.time.Time())
                
            # Calculate errors (simplified 2D navigation)
            dx = self.goal_pose.position.x - self.current_pose.position.x
            dy = self.goal_pose.position.y - self.current_pose.position.y
            distance = math.sqrt(dx**2 + dy**2)
            
            # Calculate angle to goal
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
            
        except TransformException as ex:
            self.get_logger().warn(f"TF exception: {ex}")
            return
        except Exception as ex:
            self.get_logger().error(f"Unexpected error: {ex}")
            return

    @staticmethod
    def quaternion_to_yaw(q):
        """Convert quaternion to yaw angle (simplified)"""
        # More robust conversion using all components
        t3 = +2.0 * (q.w * q.z + q.x * q.y)
        t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(t3, t4)

    @staticmethod
    def normalize_angle(angle):
        """Normalize angle to [-π, π]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    navigator = SimpleNavigator()
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot before exiting
        cmd = Twist()
        navigator.cmd_pub.publish(cmd)
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()