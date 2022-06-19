import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class StatePublisher(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('state_publisher')

        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started!".format(self.nodeName))

        loop_rate = self.create_rate(1)

        # robot state
        q1 = 1.1
        q2 = 1.2
        q3 = 1.3

        v1 = 2.1
        v2 = 2.2
        v3 = 3.3

        e1 = 3.1
        e2 = 3.2
        e3 = 3.3

        # message declarations
        joint_state = JointState()

        try:
            while rclpy.ok():
                rclpy.spin_once(self)

                # update joint_state
                now = self.get_clock().now()

                joint_state.header.stamp = now.to_msg()
                joint_state.name = ['q1', 'q2', 'q3']
                joint_state.position = [q1, q2, q3]
                joint_state.velocity = [v1, v2, v3]
                joint_state.effort = [e1, e2, e3]

                # send the joint state and transform
                self.joint_pub.publish(joint_state)

                # This will adjust as needed per iteration
                loop_rate.sleep()

        except KeyboardInterrupt:
            pass

def main():
    node = StatePublisher()

if __name__ == '__main__':
    main()
