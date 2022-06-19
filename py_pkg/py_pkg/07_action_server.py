import rclpy
from rclpy.node import Node

# Import the action server class
from rclpy.action import ActionServer

# Import the action interfaces
from custom_interfaces.action import MyAction
import time

class ActionServerNode(Node):
    def __init__(self):
        super().__init__('My_action_server')
        # Create an instance of an action server
        self.action_server = ActionServer(self, MyAction, 'my_action', self.callback)

    def callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        # Get the request using the "goal handler"
        goal = goal_handle.request.order

        # Create instances of a feedback and a result interfaces 
        result = MyAction.Result()
        feedback = MyAction.Feedback()

        feedback.partial_sequence = [0, 1]

        for i in range(1, goal):
            feedback.partial_sequence.append(feedback.partial_sequence[i] + feedback.partial_sequence[i-1])
            self.get_logger().info('Feedback: {0}'.format(feedback.partial_sequence))
            # Publish the feedback using the "goal handler"
            goal_handle.publish_feedback(feedback)
            time.sleep(1)

        # Show that the goal is now complete using the "goal handler"
        goal_handle.succeed()

        result.sequence = feedback.partial_sequence
        return result


def main(args=None):
    rclpy.init(args=args)
    action_server = ActionServerNode()
    rclpy.spin(action_server)

if __name__ == '__main__':
    main()


