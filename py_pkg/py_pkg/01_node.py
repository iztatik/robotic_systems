#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

# Create a class that inherits the imported Node class
class genericNode(Node):
    def __init__(self):
    	# Define the name of the node
        super().__init__('my_node')
        self.get_logger().info('Hello ROS2!')
        
def main(args = None ):
    # Initialize the ROS client for python
    rclpy.init(args = args)
    
    # Instance a node
    node = genericNode()
    
    # Destroy the instance of the node
    node.destroy_node()
    
    # Shutdown the ROS client 
    rclpy.shutdown()
    
    
if __name__=='__main__':
    main()
    
