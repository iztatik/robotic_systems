#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterType, ParameterDescriptor

class CustomParameter(Node):
    def __init__(self):
        super().__init__("custom_parameter_node")
        self.timer = self.create_timer(1, self.callback)

        # Parameter descriptor(optional)
        descriptor = ParameterDescriptor(description='This is a generic parameter') 

        # Parameter declaration
        self.declare_parameter('pair', True, descriptor) 


    def callback(self):
        #Get the parameter
        my_param = self.get_parameter('pair').value
        print(my_param)

        # Create a new parameter
        new_parameter = Parameter('pair', rclpy.Parameter.Type.BOOL, True) 
        
        # Set the parameters
        #self.set_parameters([new_parameter])

def main(args=None):
    try:
        rclpy.init(args=args)
        node = CustomParameter()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print()

if __name__=='__main__':
    main()

