#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped

class TFToPoseStamped(Node):
    def __init__(self):
        super().__init__('tf_to_pose_stamped')
        
        self.sub = self.create_subscription(
            TFMessage,
            '/camera/robot_pose_tf',
            self.tf_callback,
            10)
            
        self.pub = self.create_publisher(
            PoseStamped,
            '/camera/robot_pose',
            10)
    
    def tf_callback(self, msg):
        if not msg.transforms:
            return
            
        # Take the first transform (assuming single robot pose)
        tf = msg.transforms[0]
        
        pose = PoseStamped()
        pose.header = tf.header
        pose.pose.position.x = tf.transform.translation.x
        pose.pose.position.y = tf.transform.translation.y
        pose.pose.position.z = tf.transform.translation.z
        pose.pose.orientation = tf.transform.rotation
        
        self.pub.publish(pose)

def main(args=None):
    rclpy.init(args=args)
    node = TFToPoseStamped()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()