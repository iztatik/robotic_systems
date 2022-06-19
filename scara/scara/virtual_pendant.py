import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient

# Message types
from trajectory_msgs.msg import JointTrajectoryPoint
from scara_interfaces.srv import JointTrajectory
from scara_interfaces.action import TrajectoryExecution

# Modules
import tkinter as tk
from threading import Thread
import math
import time

class Teach_pendant(Node): 
    def __init__(self):
        # Node - Naming 
        super().__init__("virtual_pendant") 

        # Publisher - Point-to-point 
        self.__publisher = self.create_publisher(JointTrajectoryPoint, 'transparent_position', 10) 
        self.__timer = self.create_timer(0.1, self.__publish_point)
        self.__point = JointTrajectoryPoint()
        self.__point.positions = [0.0, 0.0, 0.0, 0.0]

        # Client - Trajectory 
        self.__client = self.create_client(JointTrajectory, 'trajectory_generator')
        while not self.__client.wait_for_service(timeout_sec=1.0): pass

        # Action-client
        self.__action_client = ActionClient(self, TrajectoryExecution, 'trajectory_execution')
        self.get_logger().info('{0} initialized...'.format(self.get_name()))

        # GUI - Root element
        self.__root = tk.Tk()
        self.__root.title('Teach pendant')
        self.__root.columnconfigure(index=0, weight=3)
        self.__root.columnconfigure(index=1, weight=1)

        # Constants 
        self.__SPEED_LIMIT = 1.0        # rad/sec

        # Variables - General
        self.__HOME = [0.0, 0.0, 0.0, 0.0]
        self.__guide_points = [self.__HOME]
        self.__trajectories = []
        self.__q = self.__HOME

        # Variables - Scales 
        self.__q_var = [tk.StringVar() for _ in range(4)]
        self.__q_range= ((-90, 90), (-90, 90), (0, .47), (-90, 90))
        self.__q_res = (1.0, 1.0, 0.01, 1.0)
        self.__speed = tk.StringVar()
        
        # GUI - Widgets
        self.__scales = [tk.Scale(self.__root, 
                                  variable = self.__q_var[i], 
                                  from_ = self.__q_range[i][0], 
                                  to = self.__q_range[i][1], 
                                  orient = tk.HORIZONTAL, 
                                  resolution = self.__q_res[i], 
                                  command = self.__set_joint_vector) 
                          for i in range(len(self.__q))]

        self.__run_bt = tk.Button(self.__root, text = 'Run', command = self.__run)
        self.__home_bt = tk.Button(self.__root, text = 'Home', command = self.__move_home)
        self.__append_bt = tk.Button(self.__root, text = 'Add', command =  self.__append_traj)
        self.__clear_bt = tk.Button(self.__root, text = 'Clear', command = self.__clear_traj)

        self.__frame = tk.LabelFrame(self.__root, text = 'Trajectories')
        self.__label = tk.Label(self.__frame, text = 'p0: [0.0, 0.0]', justify = tk.LEFT)


    def __publish_point(self):
        self.__publisher.publish(self.__point)
        return

    def __set_joint_vector(self, v):
        self.__q = [math.radians(float(val.get())) if i!=2 else float(val.get()) for i, val in enumerate(self.__q_var)]
        self.__point.positions = self.__q
        self.__home_bt['state'] = 'normal'
        return

    def __set_scales(self, val):
        for i, scale in enumerate(self.__scales):
            scale.set(val[i])
        return

    def __traj_duration(self):
        seconds = 3
        nanoseconds = 4e8
        return Duration(seconds = seconds, nanoseconds = nanoseconds)

    def __append_traj(self):
        self.__guide_points.append(self.__q) 
        self.__request_traj(self.__guide_points[-2], self.__guide_points[-1], self.__traj_duration())
        while not self.__future.done(): pass
        response = self.__future.result()
        self.__trajectories.append(response.trajectory)
        self.get_logger().info('Trajectory appended: {a}'.format(a=len(self.__trajectories))) 
 
        self.__run_bt['state'] = 'normal'
        self.__clear_bt['state'] = 'normal'
        return

    def __request_traj(self, q0, qf, duration):
        request = JointTrajectory.Request()

        request.initial_point.positions = q0 
        request.final_point.positions = qf
        request.initial_point.velocities = [0.0 for _ in range(len(q0))]
        request.final_point.velocities = [0.0 for _ in range(len(q0))]
        request.final_point.time_from_start = duration.to_msg()

        self.__future = self.__client.call_async(request)
        return

    def __clear_traj(self):
        self.__move_home()
        self.__guide_points = [self.__HOME]
        self.__trajectories = []
        self.__clear_bt['state'] = 'disabled'
        self.__run_bt['state'] = 'disabled'
        self.get_logger().info('Trajectories cleared...')
        return

    def __run(self):
        self.__timer.cancel()
        self.__run_bt['state'] = 'disabled'
        self.__home_bt['state'] = 'disabled'
        self.__move_home()
        for _, traj in enumerate(self.__trajectories):
            self.__send_goal(traj)
        self.__run_bt['state'] = 'normal'
        self.__home_bt['state'] = 'normal'
        self.__timer.reset()
        return

    def __move_home(self):
        self.get_logger().info('Moving to home...')
        self.__home_bt['state'] = 'disabled'

        self.__request_traj(self.__q, self.__HOME, self.__traj_duration())
        while not self.__future.done(): pass
        response = self.__future.result()

        self.__timer.cancel()
        self.__send_goal(response.trajectory)
        self.__timer.reset()
        self.__set_scales(self.__HOME)
        return

    def __send_goal(self, traj):
        goal_msg = TrajectoryExecution.Goal()
        goal_msg.trajectory = traj

        self.__action_client.wait_for_server()
        self.send_goal_future = self.__action_client.send_goal_async(goal_msg, 
                                                                   feedback_callback=self.__feedback_callback)
        self.send_goal_future.add_done_callback(self.__goal_response_callback)
        return

    def __goal_response_callback(self, future):
        goal_handler = future.result()

        self.get_result_future = goal_handler.get_result_async()
        self. get_result_future.add_done_callback(self.__get_result_callback)
        return

    def __get_result_callback(self, future):
        result = future.result().result
        self.__q = [math.degrees(q) if i!=2 else q for i, q in enumerate(list(result.final_point.positions))]
        self.__set_scales(self.__q)
        return

    def __feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        return
        
    def launch(self):
        n = len(self.__scales)
        for i in range(n):
            self.__scales[i].grid(row=i, column=0, sticky='ew', columnspan=2)

        self.__run_bt['state'] = 'disabled'
        self.__home_bt['state'] = 'disabled'
        self.__clear_bt['state'] = 'disabled'

        self.__run_bt.grid(row = n+1, column = 0, sticky='ew')
        self.__home_bt.grid(row = n+1, column = 1, sticky='ew') 

        self.__append_bt.grid(row = n+2, column = 0)
        self.__clear_bt.grid(row = n+2, column = 1)

        self.__root.mainloop()
        return

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
        pendant.destroy_node()
        rclpy.shutdown()
        print('')
    
if __name__ == "__main__":
    main()
