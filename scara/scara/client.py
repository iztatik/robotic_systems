#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from scara_interfaces.srv import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class Client(Node):
    def __init__(self):
        super().__init__('trajectory_client')
        self.client = self.create_client(JointTrajectory, 'trajectory_generator')

        while not self.client.wait_for_service(timeout_sec=1.0): pass

        self.request = JointTrajectory.Request()

    def my_request(self):
        self.request.initial_point.positions = [0.0, 0.0, 0.0, 0.0]
        self.request.initial_point.velocities = [0.0, 0.0, 0.0, 0.0]

        self.request.final_point.positions = [10.0, 20.0, 30.0, 40.0]
        self.request.final_point.velocities = [0.0, 0.0, 0.0, 0.0]
        self.request.final_point.time_from_start.sec = 5

        self.future = self.client.call_async(self.request)
        return


def main():
    rclpy.init()

    client = Client()
    client.my_request()
    rclpy.spin_once(client)

    if client.future.done():
        print(client.future.result().trajectory.points[10].positions)

    rclpy.shutdown()


