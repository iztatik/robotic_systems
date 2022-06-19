import rclpy
from rclpy.node import Node
from custom_interfaces.srv import MySrv

class CustomClient(Node):
    def __init__(self):
        super().__init__("My_Subscriber")

        # Client creation
        self.cli = self.create_client(MySrv, 'My_service')

        # Wait until service is ready
        while not self.cli.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info('Waiting for the service')

        # Creating a request instance
        self.req = MySrv.Request()

    # Request method
    def request(self, a, b):
        self.req.a = a
        self.req.b = b

        # Response is return as an object
        self.future = self.cli.call_async(self.req)


def main():
    rclpy.init()

    my_client = CustomClient()
    my_client.request(2, 3)
    rclpy.spin_once(my_client)

    while rclpy.ok():
        if my_client.future.done():
            try:
                response = my_client.future.result()
            except:
                my_client.get_logger().info('Response error...')
            else:
                my_client.get_logger().info(f'The result is: {response.c}')
            break

    my_client.destroy_node()
    rclpy.shutdown()

if  __name__=="__main__":
    main()
