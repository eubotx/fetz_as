import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

MAX_VELOCITY = 1.0

class GoToGoal(Node):
    def __init__(self):
        super().__init__("Go_to_Goal_Node")
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, '/pose', self.pose_callback, 10)
        self.goal_sub = self.create_subscription(Pose, '/goal_pose', self.goal_callback, 10)
        self.timer = self.create_timer(0.1, self.go_to_goal)
        
        self.pose = Pose()
        self.goal = Pose()
        self.goal_reached = True

    def pose_callback(self, data):
        self.pose = data

    def goal_callback(self, data):
        self.goal = data
        self.goal_reached = False  # Reset goal reached flag when a new goal is received
        self.get_logger().info(f"Received new goal: x={data.x}, y={data.y}, theta={data.theta}")


    def go_to_goal(self):

        if self.goal_reached:
            return  # Wait for a new goal

        new_vel = Twist()

        # Euclidean Distance
        distance_to_goal = math.sqrt((self.goal.x - self.pose.x)**2 + (self.goal.y - self.pose.y)**2)
        # Angle to Goal
        angle_to_goal = math.atan2(self.goal.y - self.pose.y, self.goal.x - self.pose.x)

        distance_tolerance = 0.1
        angle_tolerance = 0.01

        angle_error = angle_to_goal - self.pose.theta
        kp = 10

        if abs(angle_error) > angle_tolerance:
            new_vel.angular.z = kp * angle_error
        else:
            if distance_to_goal >= distance_tolerance:
                new_vel.linear.x = min(kp * distance_to_goal, MAX_VELOCITY)
            else:
                new_vel.linear.x = 0.0
                new_vel.angular.z = 0.0
                self.goal_reached = True
                self.get_logger().info("Goal Reached")
                return
            
        self.cmd_vel_pub.publish(new_vel)

def main(args=None):
    rclpy.init(args=args)
    go_to_goal_node = GoToGoal()
    rclpy.spin(go_to_goal_node)
    go_to_goal_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()