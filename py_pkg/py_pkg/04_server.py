#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_interfaces.srv import MySrv

class CustomServer(Node):
    def __init__(self):
        super().__init__("My_Server")

        # Service creation
        self.srv = self.create_service(MySrv, 'My_service', self.callback)

    def callback(self, request, response):
        response.c = request.a + request.b
        self.get_logger().info('Service requested...')
        return response

def main(args = None):
    rclpy.init(args = args)

    my_server = CustomServer()
    rclpy.spin(my_server)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
