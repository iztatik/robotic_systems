#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# Message types
from std_msgs.msg import String

# Inherited node class
class PublisherNode(Node): 
    def __init__(self):
        # Node naming
        super().__init__("my_publisher") 

        # Publisher creation
        self.publisher_ = self.create_publisher(String, 'my_topic', 10) 
        self.timer_ = self.create_timer(0.5, self.publish)
        self.get_logger().info('Publisher started...')

    # Method to publish
    def publish(self):
        my_msg = String()
        my_msg.data = "Python publisher message!"
        self.publisher_.publish(my_msg)



def main(args=None):
    # Node creation
    rclpy.init(args=args)
    my_node = PublisherNode() 
    rclpy.spin(my_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
