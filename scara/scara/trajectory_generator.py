import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterType, ParameterDescriptor

from trajectory_msgs.msg import JointTrajectoryPoint
from scara_interfaces.srv import JointTrajectory

import numpy as np

class Trajectory_generator(Node):
    def __init__(self):
        super().__init__('trajectory_generator')

        # Server
        self.server = self.create_service(JointTrajectory, 'trajectory_generator', self.server_callback)
        self.get_logger().info('{b} initialized...'.format(b=self.get_name()))

        # Parameters
        self.step = 0.02
        self.declare_parameter('time_step', self.step, ParameterDescriptor(description='Step time between poses'))

    def jTraj3(self, tf, q0, qf, v0, vf):
        # Time vector
        dt = self.step
        t = np.arange(0, tf+dt, dt)

        # Polynomia coefficients
        C0 = q0
        C1 = v0
        C2 = (3*(qf - q0)/tf**2) - ((vf + 2*v0)/tf)
        C3 = -(2*(qf - q0)/tf**3) + ((vf + v0)/tf**2)

        # Resulting trajectories
        q = np.dot(np.array([t**0, t, t**2, t**3]).transpose(), np.array([C0, C1, C2, C3]))
        v = np.dot(np.array([t**0, t, t**2]).transpose(), np.array([C1, 2*C2, 3*C3]))
        # a = np.dot(np.array([t**0, t]).transpose(), np.array([2*C2, 6*C3]))
        
        return (q, v)

    def server_callback(self, request, response):
        # Update the patrameters
        self.param_callback()

        # Generate the trajectory
        duration = float(request.final_point.time_from_start.sec)
        q0 = np.array(request.initial_point.positions)
        qf = np.array(request.final_point.positions)
        v0 = np.array(request.initial_point.velocities)
        vf = np.array(request.final_point.velocities)

        traj = self.jTraj3(duration, q0, qf, v0, vf)

        # Sort the trajetory 
        points = []

        for i in range(len(traj[0])):
            point = JointTrajectoryPoint()
            point.positions = list(traj[0][i])
            point.velocities = list(traj[1][i])
            points.append(point)

        response.trajectory.points = points

        # Update the header stamp
        now = self.get_clock().now()
        response.trajectory.header.stamp = now.to_msg()

        # Names
        response.trajectory.joint_names = ['q1', 'q2', 'q3', 'q4']

        return response

    def param_callback(self):
        self.step = self.get_parameter('time_step').value
        return


def main():
    rclpy.init()

    generator = Trajectory_generator()
    rclpy.spin(generator)

    rclpy.shutdown()

