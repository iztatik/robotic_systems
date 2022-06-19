import rclpy
from rclpy.node import Node

# Message types
from trajectory_msgs.msg import JointTrajectoryPoint

# Modules
import tkinter as tk
from threading import Thread
import math

# Inherited node class
class Teach_pendant(Node): 
    def __init__(self):
        # Node naming
        super().__init__("transparent_publisher") 

        # Publisher creation
        self.publisher = self.create_publisher(JointTrajectoryPoint, 'transparent_position', 10) 
        self.timer = self.create_timer(0.1, self.publish)
        self.point = JointTrajectoryPoint()
        self.point.positions = [0.0, 0.0, 0.0, 0.0]
        self.get_logger().info('{0} initialized...'.format(self.get_name()))

        # Root element
        self.root = tk.Tk()
        self.root.title('Scara teach pendant')
        self.root.columnconfigure(index=0, weight=3)
        self.root.columnconfigure(index=1, weight=1)

        # Joint-scales variables
        self.q = [0.0, 0.0, 0.0, 0.0]
        self.q_var = [tk.StringVar() for _ in range(4)]
        self.q_range= ((-90, 90), (-90, 90), (0, .47), (-90, 90))
        self.q_res = (1.0, 1.0, 0.01, 1.0)

        # Scales
        self.scales = [tk.Scale(self.root, 
                                variable = self.q_var[i], 
                                from_ = self.q_range[i][0], 
                                to = self.q_range[i][1], 
                                orient = tk.HORIZONTAL, 
                                resolution = self.q_res[i], 
                                command = self.set_joint_vector) 
                        for i in range(len(self.q))]

    # Method to publish
    def publish(self):
        self.publisher.publish(self.point)

    def set_joint_vector(self, v):
        self.q = [math.radians(float(val.get())) if i!=2 else float(val.get()) for i, val in enumerate(self.q_var)]
        self.point.positions = self.q

    def launch(self):
        # Scales
        for i in range(len(self.scales)):
            self.scales[i].grid(row=i, column=0, sticky='ew')
        self.root.mainloop()

def run_node(node):
    rclpy.spin(node)
    return

def main():
    try:
        rclpy.init()
        pendant = Teach_pendant()

        th1 = Thread(target = run_node, args = (pendant,))
        th1.start()

        pendant.launch()

    except(KeyboardInterrupt):
        rclpy.shutdown()
        print('')
    
if __name__ == "__main__":
    main()
