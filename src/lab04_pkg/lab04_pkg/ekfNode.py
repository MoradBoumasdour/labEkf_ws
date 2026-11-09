import rclpy
from rclpy.node import Node

import numpy as np
import math
import tf_transformations
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from math import ceil
from lab04_pkg.lab04_pkg.probabilistic_models import (
    sample_velocity_motion_model,
    landmark_range_bearing_model,
    landmark_range_bearing_sensor,
    velocity_mm_simpy,
    landmark_sm_simpy
)

from turtlebot3_perception.landmark_msgs.msg import LandmarkArray
from lab04_pkg.lab04_pkg.utils import residual
from ekf import RobotEKF

seed = 42
np.random.seed(seed)
        

class ekfNode(Node):

    def __init__(self):

        super().__init__('ekfNode')

        #data
        self.u = np.array([0.0, 0.0])
        self.ekf, self.sigma_u, self.sigma_z, self.Q_landm, self.eval_Ht = self.constructor()

        self.declare_parameter('landmarksid', list())
        self.landmarksid= self.get_parameter('landmarksid').get_parameter_value().double_array_value

        self.declare_parameter('landmarksx', list())
        self.landmarksx = self.get_parameter('landmarksx').get_parameter_value().double_array_value
        self.declare_parameter('landmarksy', list())
        self.landmarksy = self.get_parameter('landmarksy').get_parameter_value().double_array_value

        self.landmarks_dict = {
        int(l_id): [x, y]
        for l_id, x, y in zip(self.landmarksid, self.landmarksx, self.landmarksy)}

        self.declare_parameter('landmark_topic', 'landmarks')
        self.landmark_topic = self.get_parameter('landmark_topic').get_parameter_value().string_value
        #connections
        self.subscription = self.create_subscription(Odometry, 'odom',self.Odomcallback,10)
        self.subscription = self.create_subscription(LandmarkArray, self.landmark_topic, self.LandmarkCallback,10)
        #timer
        self.timer_period = 0.05
        self.timer = self.create_timer(self.timer_period, self.timer_callback)


    def constructor(self):

        #probabiistic model paramters
        dim_x = 3
        
        ######TO BE DONE######
        # general noise parameters 
        std_lin_vel = 0.08
        std_ang_vel = np.deg2rad(1.0)  # [rad/s]
        ##########################

        sigma_u = np.array([std_lin_vel, std_ang_vel])
        Mt = np.diag([std_lin_vel**2, std_ang_vel**2])
        # Velocity motion model params
        dim_u = 2
        eval_gux = sample_velocity_motion_model
        _, eval_Gt, eval_Vt = velocity_mm_simpy()
        
        
        ######TO BE DONE######
        # Define noise params and Q for landmark sensor model
        std_range = 0.1  # [m]
        std_bearing = np.deg2rad(1.0)  # [rad]
        #########################

        sigma_z = np.array([std_range, std_bearing])
        Q_landm = np.diag([std_range**2, std_bearing**2])


        # Define H Jacobian function
        _, eval_Ht = landmark_sm_simpy()

        ekf = RobotEKF(dim_x=dim_x, dim_u=dim_u, eval_gux=eval_gux, eval_Gt=eval_Gt, eval_Vt=eval_Vt)

        ########TO BE DONE########
        ekf.mu = np.array([1, 6, 0.3])  # x, y, theta #modifiare
        ekf.Sigma = np.diag([0.1, 0.1, 0.1]) #modificare
        ##########################
        ekf.Mt = Mt

        return ekf, sigma_u, sigma_z, Q_landm, eval_Ht
        
    def timer_callback(self):
       self.ekf.predict(self.u, self.sigma_u, g_extra_args=(self.timer_period,))


    def Odomcallback(self, msg):
        v = msg.twist.twist.linear
        w = msg.twist.twist.angular
        self.u = np.array([v,w])

    def LandmarkCallback(self, msg):
        for landmark in msg.landmarks:
            z = np.array([landmark.range, landmark.bearing])
            landmarkid = landmark.id
        
            self.ekf.update(z,
                                eval_hx=landmark_range_bearing_model,
                                eval_Ht=self.eval_Ht, Qt=self.Q_landm,
                                Ht_args=(*self.ekf.mu, *self.landmarks_dict[landmarkid]),
                                hx_args=(self.ekf.mu, self.landmarks_dict[landmarkid], self.sigma_z),
                                residual = residual)


if __name__ == "__main__":
    rclpy.init()
    my_node = ekfNode()
    rclpy.spin(my_node)
    my_node.destroy_node()  # cleans up pub-subs, etc
    rclpy.shutdown()

    