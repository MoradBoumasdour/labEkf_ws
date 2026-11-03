import rclpy
from rclpy.node import Node

import numpy as np
import math
import tf_transformations
from rclpy.node import Node
from  sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from math import ceil
from probabilistic_models import (
    sample_velocity_motion_model,
    landmark_range_bearing_model,
    landmark_range_bearing_sensor,
    velocity_mm_simpy,
    landmark_sm_simpy
)
import utils


from ekf import RobotEKF

seed = 42
np.random.seed(seed)
        

class ekfNode(Node):

    def __init__(self):

        super().__init__('ekfNode')

        #data
        u = np.array()
        self.ekf = self.constructor()
         #, self.sigma_u, self.sigma_z, self.Q_landm, self.eval_Ht)
     
        #connections

        self.subscription = self.create_subscription(Odometry, 'odom',self.Odomcallback,10)
        #publisher of robot's movement
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        #timer
        self.timer_period = 0.05
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        #dataudated

    def constructor():

        #sensor parameters
        mex_range = 8.0
        fov = math.pi/3

        #probabiistic model paramters
        dim_x = 3

        # general noise parameters TO BE DONE
        std_lin_vel = 0.1
        std_ang_vel = np.deg2rad(1.0)  # [rad/s]
        sigma_u = np.array([std_lin_vel, std_ang_vel])


        # Velocity motion model params
        dim_u = 2
        Mt = np.diag([std_lin_vel**2, std_ang_vel**2])
        eval_gux = sample_velocity_motion_model
        _, eval_Gt, eval_Vt = velocity_mm_simpy()

        # Define noise params and Q for landmark sensor model
        std_range = 0.1  # [m]
        std_bearing = np.deg2rad(1.0)  # [rad]
        sigma_z = np.array([std_range, std_bearing])
        Q_landm = np.diag([std_range**2, std_bearing**2])
        # Define H Jacobian function
        _, eval_Ht = landmark_sm_simpy()

        ekf = RobotEKF(dim_x=dim_x, dim_u=dim_u, eval_gux=eval_gux, eval_Gt=eval_Gt, eval_Vt=eval_Vt)
        ekf.mu = np.array([1, 6, 0.3])  # x, y, theta
        ekf.Sigma = np.diag([0.1, 0.1, 0.1])
        ekf.Mt = Mt

        return ekf#, sigma_u, sigma_z, Q_landm, eval_Ht
        
    def timer_callback(self):
       self.ekf.predict(self.u, self.sigma_u, g_extra_args=(self.timer_period))
       msg = Twist()
       Twist.linear = self.ekf.mu[0]
       Twist.angular = self.ekf.mu[1]
       



        

    def Odomcallback(self, msg):
        v = msg.Twist.Twist.linear
        w = msg.Twist.Twist.angular
        self.u = np.array(v,w)
        self.sigma_u = msg.Twist.covariance

if __name__ == "__main__":
    rclpy.init()
    my_node = ekfNode()
    rclpy.spin(my_node)
    my_node.destroy_node()  # cleans up pub-subs, etc
    rclpy.shutdown()