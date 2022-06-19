import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import numpy as np

class Trajectory_publisher(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('state_publisher')

        self.joint_publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.node_name = self.get_name()
        self.get_logger().info("{0} started!".format(self.node_name))

        loop_rate = self.create_rate(1)

        # Message declarations
        joint_state = JointState()

        # Initial state 
        self.counter = float('inf')
        self.traj = ((0, 0),(0, 0))
        joint_state.name = ['q1', 'q2', 'q3', 'q4']
        joint_state.position = [0.0, 0.0, 0.0, 0.0]
        joint_state.velocity = [0.0, 0.0, 0.0, 0.0]

        try:
            while rclpy.ok():
                rclpy.spin_once(self)

                # Update the header stamp
                now = self.get_clock().now()
                joint_state.header.stamp = now.to_msg()

                # Update the joins state
                if counter <= len(traj[0]):
                    joint_state.position = trajectory[0][counter]
                    joint_state.velocity = trajectory[1][counter]
                    counter =+ 1

                self.joint_publisher.publish(joint_state)

                loop_rate.sleep()

        except KeyboardInterrupt:
            pass

    def jTraj3(self, tf, q0, qf, v0, vf):
        # Time vector
        dt = 0.2
        t = np.arange(0, tf+dt, dt)

        # Polynomia coefficients
        C0 = q0
        C1 = v0
        C2 = (3*(qf - q0)/tf**2) - ((vf + 2*v0)/tf)
        C3 = -(2*(qf - q0)/tf**3) + ((vf + v0)/tf**2)

        # Resulting trajectories
        q = np.dot(np.array([t**0, t, t**2, t**3]).transpose(), np.array([C0, C1, C2, C3]))
        v = np.dot(np.array([t**0, t, t**2]).transpose(), np.array([C1, 2*C2, 3*C3]))
        
        return (q, v)

    def set_traj(self, pt1, pt2):
        pass


def main():
    node = Trajectory_publisher()

if __name__ == '__main__':
    main()
