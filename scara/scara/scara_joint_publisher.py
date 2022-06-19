import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.duration import Duration

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from scara_interfaces.srv import JointTrajectory
from scara_interfaces.action import TrajectoryExecution

import time

class Trajectory_publisher(Node):
    def __init__(self):
        super().__init__('scara_joint_publisher')

        # Joint_state_publisher
        self.joint_publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.joint_publish)
        self.node_name = self.get_name()

        # Transparent_position_subscriber
        self.joint_subscriber = self.create_subscription(JointTrajectoryPoint, 
                                                        'transparent_position',
                                                         self.get_point ,10)

        # Client - Trajectory 
        self.__client = self.create_client(JointTrajectory, 'trajectory_generator')
        while not self.__client.wait_for_service(timeout_sec=1.0): pass
        self.get_logger().info('{0} initialized...'.format(self.get_name()))

        # Action-server
        self.__action_server = ActionServer(self, TrajectoryExecution, 'trajectory_execution', self.__traj_execution) 


        ## Messages declarations
        self.joint_state = JointState()

        # Initial state 
        self.counter = float('inf')
        self.trajectory = ((0, 0),(0, 0))
        self.joint_state.name = ['q1', 'q2', 'q3', 'q4']
        self.joint_state.position = [0.0, 0.0, 0.0, 0.0]
        self.joint_state.velocity = [0.0, 0.0, 0.0, 0.0]

        # Start publisher
        self.joint_publish()

    def joint_publish(self):
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.joint_publisher.publish(self.joint_state)
        return

    def get_point(self, point):
        self.joint_state.position = point.positions
        return

    def __traj_execution(self, goal_handler):
        trajectory = goal_handler.request.trajectory
        result = TrajectoryExecution.Result()

        self.destroy_subscription(self.joint_subscriber)

        feedback_msg = TrajectoryExecution.Feedback()
        result_msg = TrajectoryExecution.Result()

        for i, point in enumerate(trajectory.points):
            self.joint_state.header.stamp = self.get_clock().now().to_msg()
            self.joint_state.position = point.positions
            self.joint_publisher.publish(self.joint_state)
            feedback_msg.actual_point = point
            goal_handler.publish_feedback(feedback_msg)
            time.sleep(0.02)

        goal_handler.succeed()
        self.joint_subscriber = self.create_subscription(JointTrajectoryPoint, 
                                                        'transparent_position',
                                                         self.get_point ,10)

        result_msg.final_point = trajectory.points[-1]
        return result_msg 


def main():
    rclpy.init()
    node = Trajectory_publisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
