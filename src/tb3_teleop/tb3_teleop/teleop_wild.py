# import tty
# import sys
# import time
# import termios
# import numpy as np
# from queue import Queue
# from select import select
# from threading import Thread
# from functools import partial
# import matplotlib.pyplot as plt
# from scipy.interpolate import splev, splder, splrep

# import rclpy
# from rclpy.qos import QoSProfile
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Twist
# from tf_transformations import euler_from_quaternion



# class ConvoyControl():
#     def __init__(self, num_bots, vel_max, y, a, b):
        
#         self.y = y
#         self.a = a
#         self.b = b
#         self.vel_max = vel_max
        
#         self.t = 0
#         self.xt = lambda t : 0
#         self.d_xt = lambda t : 0
#         self.d2_xt = lambda t : 0

#         self.yt = lambda t : 0
#         self.d_yt = lambda t : 0
#         self.d2_yt = lambda t : 0

#         self.R = np.eye(2) # R_ab 
#         self.revang=0 # Reverse angle (theta_ab)
#         self.angle_change=np.pi*1/6 # Default frame change

#         self.num_bots = num_bots
#         self.curr_vel = np.zeros((num_bots, 2))
#         self.desired_pose = np.zeros((num_bots, 3))
#         self.error = np.zeros_like(self.desired_pose)
        
#         self.desired_pose[:, 0] += np.arange(num_bots-1, -1, -1)
#         # self.desired_pose[:, 1] = self.gx(self.desired_pose[:, 0])
#         # self.desired_pose[:, 2] = np.arctan(self.d_gx(self.desired_pose[:, 0]))

#         self.curr_pose = np.copy(self.desired_pose)
#         # self.curr_pose *= (1-0.25*np.random.rand(num_bots, 3))
#         # self.desired_pose[:, 0] += 0.1; self.curr_pose[0, 0] = self.desired_pose[0, 0]
        
#         super().__init__()
#         self.pub_vel_nodes = []
#         self.odom_data_nodes = []
#         qos = QoSProfile(depth = 10)

#         for i in range(self.num_bots):
#             cmd_vel_node = rclpy.create_node(f"teleop_{i}")
#             self.pub_vel_nodes.append(cmd_vel_node.create_publisher(Twist, f"/tb3_{i}/cmd_vel", qos))

#             self.odom_data_nodes.append(rclpy.create_node(f"get_pose_{i}"))
#             self.odom_data_nodes[-1].create_subscription(Odometry, f"/tb3_{i}/odom", partial(self.get_odom_data, i), qos)
        
#         self.last_time = time.time()

    
#     def get_odom_data(self, bot_index, msg):
#         _, _, yaw = euler_from_quaternion([msg.pose.pose.orientation.x,
#                                            msg.pose.pose.orientation.y,
#                                            msg.pose.pose.orientation.z,
#                                            msg.pose.pose.orientation.w])
        
#         self.curr_pose[bot_index] =  np.array([msg.pose.pose.position.x,
#                                                msg.pose.pose.position.y, yaw])
#         # Rotate X,Y as (R_ab @ [[x],[y]]).T
#         self.curr_pose[bot_index, 0:2] =  self.curr_pose[bot_index, 0:2]@self.R.transpose()
#         # Turn back the theta
#         self.curr_pose[bot_index, 2] -=self.revang

#     def get_derivative_second_deriative(self, t):
#         D_xt=self.d_xt(t)
#         D_yt=self.d_yt(t)
#         D_yx=D_yt/D_xt
        

#     def get_curr_err(self):
#         for i in range(self.num_bots):
#             rclpy.spin_once(self.odom_data_nodes[i])

#         dt = time.time() - self.last_time
#         self.last_time = time.time()


#         self.desired_pose[0, 0] = self.curr_pose[0, 0]
#         self.desired_pose[1:, 0] += self.vel_max*dt/np.sqrt(1 + np.square(self.d_gx(self.desired_pose[1:, 0])))
        
#         self.desired_pose[:, 1] = self.gx(self.desired_pose[:, 0])
#         self.desired_pose[:, 2] = np.arctan(self.d_gx(self.desired_pose[:, 0]))
#         self.error = np.copy(self.curr_pose - self.desired_pose)
    
    
#     def get_ctrl_input(self):
#         self.curr_vel[0, 0] = self.vel_max
    
#         self.curr_vel[1:, 0] = ((self.curr_vel[0, 0]
#                                 *(np.cos(self.desired_pose[1:, -1])
#                                   -self.y*self.error[1:, 0])
#                                 /np.cos(self.curr_pose[1:, -1])))
            
#         self.curr_vel[:, 1] = (self.curr_vel[:, 0]*self.d2_gx(self.curr_pose[:, 0])
#                                /np.power(1 + np.square(self.d_gx(self.curr_pose[:, 0])), 1.5)
#                                - self.a*self.error[:, 1] - self.b*self.error[:, 2])
        
#     def update_rotation_matrix(self):
#         # Function to update default rotation value on frame change
#         self.R=np.array([[np.cos(self.angle_change), -np.sin(self.angle_change)],[np.sin(self.angle_change), np.cos(self.angle_change)]])
#         self.revang=self.angle_change

#     def update_traj(self, sign):
#         # Update the goal position
#         goal_angle = self.curr_pose[0, 2] + sign*1e-1
    
#         # Apply rotation if the goal angle exceeds threshold
#         if np.abs(goal_angle) > 0.80 * np.pi / 2:
#                 self.update_rotation_matrix()
#                 self.desired_pose[:, 0:2] = self.desired_pose[:, 0:2] @ self.R.transpose()
#                 self.desired_pose[:, 2] = np.arctan(self.d_gx(self.desired_pose[:, 0]))
#                 goal_angle=goal_angle-self.revang
            
#         # Design the goal        
#         self.goal = np.array([np.cos(goal_angle), np.sin(goal_angle)])@self.R.transpose() + self.desired_pose[0, :2]

#         # Position of all bots and goal 
#         x_coord = np.concatenate((self.desired_pose[::-1, 0], [self.goal[0]]))
#         y_coord = np.concatenate((self.desired_pose[::-1, 1], [self.goal[1]]))

#         try: 
#             spline_tup = splrep(x_coord, y_coord, s=0)
#             spline_tup_der = splder(spline_tup, n=1)
#             spline_tup_der2 = splder(spline_tup, n=2)
        
#             self.gx = lambda x : splev(x, spline_tup)
#             self.d_gx = lambda x : splev(x, spline_tup_der)
#             self.d2_gx = lambda x : splev(x, spline_tup_der2)
    
#         except Exception as e:
#                 print("Exception:", e)
#                 print("x_coord:", x_coord)
#                 print("y_coord:", y_coord)
#                 plt.plot(x_coord, y_coord, marker='o', linestyle='-', label="Failed spline data")
#                 plt.legend()
#                 plt.show()
    
#     def ctrl_bots(self):
#         for i in range(self.num_bots):
#             twist = Twist()

#             twist.linear.x = self.curr_vel[i,0]
#             twist.linear.y = 0.0
#             twist.linear.z = 0.0

#             twist.angular.x = 0.0
#             twist.angular.y = 0.0
#             twist.angular.z = self.curr_vel[i,1]
            
#             self.pub_vel_nodes[i].publish(twist)


#     def step_sim(self):
#         self.get_curr_err()
#         self.get_ctrl_input()
#         self.ctrl_bots()


# global key; key = ''
# def keyboard_listener(timeout=0.5):
#     global key
#     while True:
#         tty.setraw(sys.stdin.fileno())
#         rlist, _, _ = select([sys.stdin], [], [], timeout)
#         key = sys.stdin.read(1) if rlist else ''
#         termios.tcsetattr(sys.stdin, termios.TCSADRAIN,
#                             termios.tcgetattr(sys.stdin))
#         if (key == '\x03'): return


# def main():
#     num_bots = number_of_bots()
#     convoy_params = {"num_bots": num_bots,
#                      "vel_max": 0.5,
#                      "y": 10.0,
#                      "a": 4.5,
#                      "b": 7.5}
    
#     rclpy.init()
#     C = ConvoyControl(**convoy_params)

#     global key
#     key_dict = {'d': -1, 'a': 1, 'w': 0, 's': 0}
#     input_thread = Thread(target=keyboard_listener)
#     input_thread.daemon = True
#     input_thread.start()

#     pos_err = []
#     cmd_vel = []
#     ang_err = []
    
#     while(1):
#         if key in key_dict.keys():
#             if key=='w':
#                 C.vel_max*=1.01
#             if key=='s':
#                 C.vel_max*=0.97
#             C.update_traj(key_dict[key])
#             print(f'{key}', end='\r')
            
#         if (key == '\x03'): break

#         C.step_sim()
#         pos_err.append(np.linalg.norm(C.error[:,:2], axis=1))
#         cmd_vel.append(np.copy(C.curr_vel[:,0]))
#         ang_err.append(np.copy(C.error[:,2]))

        
#     pos_err = np.array(pos_err)
#     cmd_vel = np.array(cmd_vel)
#     ang_err = np.array(ang_err)

#     _, ax = plt.subplots(3, 1, figsize=(15, 20))

#     for i in range(num_bots):
#         j = 0
#         ax[j].plot(pos_err[:,i])
#         ax[j].set_ylabel("Error in Postion (m)")
#         j += 1

#         ax[j].plot(ang_err[:,i])
#         ax[j].set_ylabel("Error in Angular (radians)")
#         j += 1

#         ax[j].plot(cmd_vel[:,i])
#         ax[j].set_ylabel("Command Velocity (m/s)")
#         ax[j].set_xlabel("Time (s)")
#         j += 1

#     plt.show()

#     rclpy.shutdown()
#     sys.exit()
#     return

# def number_of_bots():
#     return 5

# if __name__ == '__main__':
#     main()