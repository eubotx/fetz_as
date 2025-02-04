import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates

class GroundTruthPosePublisher(Node):
    def __init__(self):
        super().__init__('ground_truth_pose_publisher')
        self.model_name = 'differential_drive_robot'  # Replace with your robot's model name in Gazebo
        self.pose_publisher = self.create_publisher(PoseStamped, 'robot_pose', 10)
        self.create_subscription(ModelStates, '/gazebo/model_states', self.callback, 10)

    def callback(self, msg):
        if self.model_name in msg.name:
            index = msg.name.index(self.model_name)
            pose = msg.pose[index]

            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.header.frame_id = 'world'
            pose_stamped.pose = pose

            self.pose_publisher.publish(pose_stamped)
        else:
            self.get_logger().warn(f"Model {self.model_name} not found in /gazebo/model_states")

def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
