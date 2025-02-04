# Python publisher node
# Author: Florian Geissler
# Date: January 05 2024

# rclpy is a Python API for communicating and interacting with ROS2
import rclpy

from std_msgs.msg import String

from rclpy.node import Node

# create  publisher node inherating from node
class PublisherNode(Node):

    def __init__(self):
        # take constructor of parent class and specify name
        super().__init__('node_publisher')

        # declare node publish message type
        # specify topic name
        # queue size for buffering the message
        self.publisher_ = self.create_publisher(String, 'communication_topic', 15)

        # communication rate in hertz
        commRate = 1

        # timer to call the callback function at given commRate
        self.timer = self.create_timer(commRate, self.callbackFunction)

        # counter
        self.counter = 0

    def callbackFunction(self):
        # create message
        messagePublisher = String()

        # fill message with data
        messagePublisher.data = 'Counter value: %d' % self.counter

        # publish message to topic
        self.publisher_.publish(messagePublisher)

        # publish also in logger to show in cli window running node
        self.get_logger().info('Publisher node is publishing: "%s"' % messagePublisher.data)

        # increment counter
        self.counter = self.counter+1

def main(args=None):
    rclpy.init(args=args)
    node_publisher = PublisherNode()
    # spin function to spin node and therefore activate callbacks
    rclpy.spin(node_publisher)
    node_publisher.destroy_node()
    rclpy.shutdown()

if __name__== '__main__':
    main()