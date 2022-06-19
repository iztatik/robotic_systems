#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# Messages types
from std_msgs.msg import String

class MySubscriber(Node): 
    def __init__(self):
        super().__init__('my_subscriber') 

        # Create a subscriber
        self.subscriber_ = self.create_subscription(String, 'my_topic', self.callback, 10)
        self.get_logger().info('Subscriber created')

    def callback(self, my_msg):
        self.get_logger().info(my_msg.data)


def main(args=None):
    # Create a node
    rclpy.init(args=args)
    node = MySubscriber() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
