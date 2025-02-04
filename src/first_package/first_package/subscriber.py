# Python subscriber node
# Author: Florian Geissler
# Date: January 06 2024

# rclpy is a Python API for communicating and interacting with ROS2
import rclpy

from std_msgs.msg import String

from rclpy.node import Node

class SubscriberNode(Node):

    def __init__(self):
        # parent constructor and specify name
        super().__init__('node_subscriber')

        # receive String
        # specify topic name to listen to
        # specify callback function
        # specify buffer size
        self.subscription=self.create_subscription(String, 'communication_topic', self.callbackFunction, 15)

        # prevent unused variable warning
        self.subscription

    def callbackFunction(self, message):
        self.get_logger().info('We received: "%s"' % message.data)

def main(args=None):
    rclpy.init(args=args)

    node_subscriber = SubscriberNode()

    rclpy.spin(node_subscriber)

    node_subscriber.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()