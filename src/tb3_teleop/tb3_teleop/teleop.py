import tty
import sys
import time
import termios
import numpy as np
from queue import Queue
from select import select
from threading import Thread
from functools import partial
import matplotlib.pyplot as plt
from scipy.interpolate import splev, splder, splrep
from scipy.misc import derivative
import os
import importlib.util
import logging

import rclpy
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion

logging.basicConfig(
    filename='my_log_file.log',  # Log file name
    filemode='w',               # Overwrite ('w') or append ('a')
    level=logging.INFO,         # Log level
    format='%(asctime)s - %(levelname)s - %(message)s'
)


class ConvoyControl():
    def __init__(self):
        
        config = self.load_config()

        # Initialize logging
        self.logger = self.initialize_logger()
        self.logger.info("Simulation started.")
        
        # Parameters for control and trajectory generation
        self.y = config.y # Gamma parameter
        self.a = config.a # a parameter
        self.b = config.b # b parameter
        self.linear_vel = config.linear_vel # linear velocitiy
        self.num_bots = config.num_bots # number of bots
        
        # Time parametrized function of x and y 
        self.xt = config.xt # x = xt(t)
        self.yt = config.yt # y = yt(t)
        self.d_xt = config.d_xt # first derivative of x w.r.t t
        self.d_yt = config.d_yt # first derivative of y w.r.t t

        self.dy_dx = lambda t : self.d_yt(t)/self.d_xt(t) # first derivative of trajector w.r.t x
        self.d2y_dx2 =lambda t : derivative(self.dy_dx, t, dx=1e-6) / self.d_xt(t) # second derivative of trajectory w.r.t x
        
        self.dt = config.dt # Time step

        self.t = config.compute_t(self.num_bots)# time varaible of each bot

        # Frame direction correction varaibles
        self.rot_ang = 0 # Reverse angle (theta_ab)
        self.angle_change = config.angle_change # Default frame change
        self.R = np.eye(2) # R_ab 
        self.R_ = np.eye(2)

        self.curr_vel = np.zeros((self.num_bots, 2)) # current velocity of bots [linear, angular]    
        
        # desired positions of bots [x*, y*, theta*].T
        self.desired_pose = np.zeros((self.num_bots, 3))
        self.desired_pose[:, 0] = self.xt(self.t)
        self.desired_pose[:, 1] = self.yt(self.t)
        self.desired_pose[:, 2] = np.arctan(self.dy_dx(self.t))

        # current positions of bots [x, y, theta].T
        self.curr_pose = np.copy(self.desired_pose)
        
        self.error = np.zeros_like(self.desired_pose) # position error for bots 
        
        super().__init__()
        self.pub_vel_nodes = []
        self.odom_data_nodes = []
        qos = QoSProfile(depth = 10)

        for i in range(self.num_bots):
            cmd_vel_node = rclpy.create_node(f"teleop_{i}")
            self.pub_vel_nodes.append(cmd_vel_node.create_publisher(Twist, f"/tb3_{i}/cmd_vel", qos))

            self.odom_data_nodes.append(rclpy.create_node(f"get_pose_{i}"))
            self.odom_data_nodes[-1].create_subscription(Odometry, f"/tb3_{i}/odom", partial(self.get_odom_data, i), qos)
        
        self.last_time = time.time()

    def load_config(self):
        # Locate and load the config Python file from the grandparent directory
        current_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(current_dir, '..', '..', 'config.py')

        if not os.path.exists(config_path):
            raise FileNotFoundError(f"Configuration file not found at {config_path}")

        spec = importlib.util.spec_from_file_location("config", config_path)
        config = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(config)
        return config
    
    def initialize_logger(self):
        log_dir = "logs"
        os.makedirs(log_dir, exist_ok=True)
        logger = logging.getLogger("ConvoyControlLogger")
        logger.setLevel(logging.INFO)

        handler = logging.FileHandler(os.path.join(log_dir, "simulation.log"))
        formatter = logging.Formatter("%(asctime)s - %(message)s")
        handler.setFormatter(formatter)
        logger.addHandler(handler)

        return logger

    def log_step(self):
        for i in range(self.num_bots):
            self.logger.info(
                f"Bot {i}: curr_pose={self.curr_pose[i]}, "
                f"desired_pose={self.desired_pose[i]}, "
                f"rot_angle={self.rot_ang:.4f}, "
                f"error={self.error[i]}"
            )
    
    def update_rotation_matrix(self):
        """
        Updates the default frame rotation matrix R_ab, and rotation angle whenever it is called.

        This function calculates the rotation matrix and rotation angle based on angle_change stored in the class instance.

        Parameters:
        None. The function uses the angle_change 
        simulation parameters stored in the class instance.

        Returns:
        None. 
        """
        # Function to update default rotation value on frame change
        self.angle_change = np.copysign(self.angle_change, self.curr_pose[0,2])
        self.rot_ang += self.angle_change
        
        self.R = np.array([[np.cos(self.rot_ang), -np.sin(self.rot_ang)],
                         [np.sin(self.rot_ang), np.cos(self.rot_ang)]])
        
        self.R_ = np.array([[np.cos(self.angle_change), -np.sin(self.angle_change)],
                         [np.sin(self.angle_change), np.cos(self.angle_change)]])

    def get_odom_data(self, bot_index, msg):
        """
        Updates the current position of the robots.

        This function calculates the current position of each robot in the simulation,
        updates the current position based on the rotation matrix.

        Parameters:
        None. The function uses the bot index, message object, and
        simulation parameters stored in the class instance.

        Returns:
        None. 
        """
        _, _, yaw = euler_from_quaternion([msg.pose.pose.orientation.x,
                                           msg.pose.pose.orientation.y,
                                           msg.pose.pose.orientation.z,
                                           msg.pose.pose.orientation.w])
        
        self.curr_pose[bot_index] =  np.array([msg.pose.pose.position.x,
                                               msg.pose.pose.position.y, yaw])
        # Rotate X,Y as (R_ab @ [[x],[y]]).T
        self.curr_pose[bot_index, 0:2] =  self.curr_pose[bot_index, 0:2] @ self.R

        # if self.curr_pose[bot_index, 2] > np.pi * 2:
            # self.curr_pose -= np.pi * 2
        # Turn back the theta
        self.curr_pose[bot_index, 2] -= self.rot_ang


    def get_curr_err(self):
        """
        Reads the current position and calcaulate the desired positions, and error of the robots.

        This function reads the current position of each robot in the simulation,
        updates the desired position based on the trajectory functions, and then calculates
        the error between the current and desired positions.

        Parameters:
        None. The function uses the current state of the robots, trajectory functions, and
        simulation parameters stored in the class instance.

        Returns:
        None. The function updates the 'curr_pose', 'desired_pose', and 'error' attributes
        of the class instance.
        """
        for i in range(self.num_bots):
            rclpy.spin_once(self.odom_data_nodes[i])
        
        # Apply rotation if the goal angle exceeds threshold
        if np.abs(self.curr_pose[0, 2]) > 0.3 * np.pi:
            self.update_rotation_matrix()
            # Rotate X,Y as (R_ab @ [[x],[y]]).T
            self.curr_pose[:, 0:2] =  self.curr_pose[:, 0:2] @ self.R_
            # Turn back the theta
            self.curr_pose[:, 2] -= self.angle_change#rot_ang
            # Update desired trajectory
            self.desired_pose[:, 0:2] =  self.desired_pose[:, 0:2] @ self.R_
            # Turn back the theta
            self.desired_pose[:, 2] -= self.angle_change
            self.desired_pose[:, 2] = np.where(
                    self.desired_pose[:, 2] < 0, 
                    self.desired_pose[:, 2] + np.pi, 
                    self.desired_pose[:, 2]
                    )
            self.update_traj()

        self.dt = time.time() - self.last_time
        self.last_time = time.time()

        # Update the desired pose and orientation for virtual bots
        self.desired_pose[:, 0] = self.xt(self.t)
        self.desired_pose[:, 1] = self.yt(self.t)
        self.desired_pose[:, 2] = np.arctan(self.dy_dx(self.t))
        
        self.error = np.copy(self.curr_pose - self.desired_pose)
        self.log_step()
        
        if np.any(self.error > 1):
            print(self.rot_ang)
            # Plot the X-Y components
            plt.figure(figsize=(10, 5))  # Set figure size
            plt.subplot(2, 1, 1)  # First subplot (top)
            plt.plot(self.curr_pose[:, 0],self.curr_pose[:,1], 'x--', label='Current Pose')
            plt.plot(self.desired_pose[:, 0] ,self.desired_pose[:, 1], 'o--', label='Desired Pose')
            plt.title('X-Y Components')
            plt.xlabel('Index')
            plt.ylabel('Position')
            plt.legend()
            plt.grid(True)

            # Plot the Z component
            plt.subplot(2, 1, 2)  # Second subplot (bottom)
            plt.plot(self.curr_pose[:, 2], 'x--', label='Current Pose')
            plt.plot(self.desired_pose[:, 2], 'o--', label='Desired Pose')
            plt.title('Z Component')
            plt.xlabel('Index')
            plt.ylabel('Position')
            plt.legend()
            plt.grid(True)

            plt.tight_layout()
            image_path = "pose_comparison.png"  # Path to save the image
            plt.savefig(image_path)
            print(f"Figure saved at: {image_path}")

            # Show the plot
            plt.show()

            # Exit the program
            sys.exit()
        

    
    def get_ctrl_input(self):
        """
        Calculates the control inputs (linear and angular velocities) for each robot based on the current error and trajectory.

        Parameters: None. the function only uses the instance of the class, and its arributes such as the linear velocity, the gamma parameter, 
        the current position error of the robots, the desired position of the robots, the current position of the robots, 
        the first derivative of the trajectory function g(x), the second derivative of the trajectory function g(x), the a parameter,
        the b parameter.

        Returns:
        None. The function updates the 'curr_vel' attribute of the class instance with the calculated control inputs.
        """

        self.curr_vel[:, 0] = ((self.linear_vel * np.cos(self.desired_pose[:, 2]) - self.y * self.error[:, 0])
                               / np.cos(self.curr_pose[:, 2]))
            
        self.curr_vel[:, 1] = (self.curr_vel[:, 0] * (self.d2y_dx2(self.t))
                               / np.power(1 + np.square(self.dy_dx(self.t)), 1.5)
                               - self.a * self.error[:, 1] - self.b * self.error[:, 2])
        
        self.t += self.linear_vel*self.dt/(np.sqrt(np.square(self.d_xt(self.t)) + np.square(self.d_yt(self.t))))
        
    

    def update_traj(self, sign=0):
        """
        Updates the trajectory functions of the robots based on the sign i.e. the direction you want to turn.

        Parameters:
        sign (int): The sign indicating the direction of the trajectory update.
            A positive sign indicates a rightward trajectory update, while a negative sign indicates a leftward trajectory update.

        Returns:
        None. The function updates the trajectory functions ().
        """
        # self.t = 2.*np.arange(self.num_bots-1, -1, -1) + 1.
        if sign in [-1,1]:
            # Update the goal position
            goal_angle = self.curr_pose[0, 2] + sign*2e-1
            self.goal = (10.*np.array([np.cos(goal_angle),
                                      np.sin(goal_angle)])
                         + self.desired_pose[0, :2])
            
            t_span = np.concatenate((self.t[::-1], [self.t[0] + 5.]))
            x_coord = np.concatenate((self.desired_pose[::-1, 0], [self.goal[0]]))
            y_coord = np.concatenate((self.desired_pose[::-1, 1], [self.goal[1]]))

        else:
            t_span = self.t[::-1]
            x_coord = self.desired_pose[::-1, 0]
            y_coord = self.desired_pose[::-1, 1]
            

        try: 
            spline_x = splrep(t_span, x_coord, s=0)
            spline_der_x = splder(spline_x, n=1)
            self.xt = lambda t : splev(t, spline_x)
            self.d_xt = lambda t : splev(t, spline_der_x)
            
        except Exception as e:
                print("Exception:", e)
                print("x_coord:", x_coord)
                print("t_span:",t_span)
                plt.plot(x_coord, t_span, marker='o', linestyle='-', label="Failed X spline data")
                plt.legend()
                plt.show()
        
        try:
            spline_y = splrep(t_span, y_coord, s=0)
            spline_der_y = splder(spline_y, n=1)
            self.yt = lambda t : splev(t, spline_y)
            self.d_yt = lambda t : splev(t, spline_der_y)

        except Exception as e:
                print("Exception:", e)
                print("y_coord:", y_coord)
                print("t_span:",t_span)
                plt.plot(y_coord, t_span, marker='o', linestyle='-', label="Failed Y spline data")
                plt.legend()
                plt.show()

        try:
            self.dy_dx = lambda t : self.d_yt(t) / self.d_xt(t)
            self.d2y_dx2 = lambda t : derivative(self.dy_dx, t, dx=1e-6) / self.d_xt(t)
        
        except Exception as e:
                print("Exception:", e)

    
    def ctrl_bots(self):
        for i in range(self.num_bots):
            twist = Twist()

            twist.linear.x = self.curr_vel[i,0]
            twist.linear.y = 0.0
            twist.linear.z = 0.0

            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = self.curr_vel[i,1]
            
            self.pub_vel_nodes[i].publish(twist)


    def step_sim(self):
        self.get_curr_err()
        self.get_ctrl_input()
        self.ctrl_bots()


global key; key = ''
def keyboard_listener(timeout=0.5):
    global key
    while True:
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select([sys.stdin], [], [], timeout)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN,
                            termios.tcgetattr(sys.stdin))
        if (key == '\x03'): return


def main():
    rclpy.init()
    C = ConvoyControl()
    global key
    key_dict = {'d': -1, 'a': 1, 'w': 0, 's': 0}
    input_thread = Thread(target=keyboard_listener)
    input_thread.daemon = True
    input_thread.start()

    pos_err = []
    cmd_vel = []
    ang_err = []
    
    while(1):
        if key in key_dict.keys():
            # if key=='w':
            #     C.linear_vel*=1.01
            # if key=='s':
            #     C.linear_vel*=0.97
            C.update_traj(key_dict[key])
            print(f'{key}', end='\r')
            
        if (key == '\x03'): break
        C.step_sim()
        pos_err.append(np.linalg.norm(C.error[:,:2], axis=1))
        cmd_vel.append(np.copy(C.curr_vel[:,0]))
        ang_err.append(np.copy(C.error[:,2]))

        
    pos_err = np.array(pos_err)
    cmd_vel = np.array(cmd_vel)
    ang_err = np.array(ang_err)

    _, ax = plt.subplots(3, 1, figsize=(15, 20))

    for i in range(C.num_bots):
        j = 0
        ax[j].plot(pos_err[:,i])
        ax[j].set_ylabel("Error in Postion (m)")
        j += 1

        ax[j].plot(ang_err[:,i])
        ax[j].set_ylabel("Error in Angular (radians)")
        j += 1

        ax[j].plot(cmd_vel[:,i])
        ax[j].set_ylabel("Command Velocity (m/s)")
        ax[j].set_xlabel("Time (s)")
        j += 1

    plt.show()

    rclpy.shutdown()
    sys.exit()
    return


if __name__ == '__main__':
    main()