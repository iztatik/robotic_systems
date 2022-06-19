import rclpy
from rclpy.node import Node

# Import the action client class
from rclpy.action import ActionClient

from custom_interfaces.action import MyAction

class ActionClientNode(Node):
    def __init__(self):
        super().__init__('my_action_client')
        # Create an instance of an action client
        self.action_client = ActionClient(self, MyAction, 'my_action')

    def send_goal(self, order):
        # Create an instance of the goal interface
        goal = MyAction.Goal()
        goal.order = order

        # Wait for the server to be available
        self.action_client.wait_for_server()

        # Send the goal to the server and get a "future goal object"
        self.send_goal_future = self.action_client.send_goal_async(goal)

        # Associate the "future goal object" to a callback method for when the server accepts/rejects the goal
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        # Instance a "goal handler" using the "future goal object"
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self.get_logger().info('Goal accepted')

        # Request the result and get a "future result object" using the "goal handler"
        self.get_result_future = goal_handle.get_result_async()

        # Associate the "future result object" to a callback method for when the result is ready
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        # Get the result
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
        # Shutdown the node
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    action_client = ActionClientNode()

    action_client.send_goal(5)

    rclpy.spin(action_client)

if __name__ == '__main__':
    main()

