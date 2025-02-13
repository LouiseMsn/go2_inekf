#!/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from unitree_go.msg import LowState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

import numpy as np
import math

# ==============================================================================
# Main Class
# ==============================================================================

class Bias_calc(Node):

    def __init__(self):
        super().__init__('bias_calc')
        # ROS2 =================================================================
        self.subscription_ = self.create_subscription(LowState,
                                                    '/lowstate',
                                                    self.listener_callback,
                                                    10)
        
        self.nb_samples  = 0
        self.res_acc = np.zeros((3,1))
        self.res_gyro = np.zeros((3,1))
        self.res_norm = 0
        

    


    def listener_callback(self, state_msg):
        self.nb_samples += 1
        # IMU measurement ======================================================
        # self.imu_measurement_[0][0] = state_msg.imu_state.gyroscope[0]
        # self.imu_measurement_[1][0] = state_msg.imu_state.gyroscope[1]
        # self.imu_measurement_[2][0] = state_msg.imu_state.gyroscope[2]

        # Accelerometer ========================================================
        self.res_acc[0][0] = self.res_acc[0][0]*((self.nb_samples-1)/self.nb_samples) + state_msg.imu_state.accelerometer[0]/self.nb_samples
        self.res_acc[1][0] = self.res_acc[1][0]*((self.nb_samples-1)/self.nb_samples) + state_msg.imu_state.accelerometer[1]/self.nb_samples
        self.res_acc[2][0] = self.res_acc[2][0]*((self.nb_samples-1)/self.nb_samples) + state_msg.imu_state.accelerometer[2]/self.nb_samples

        # Gyroscope ============================================================
        self.res_gyro[0] = self.res_gyro[0]*((self.nb_samples-1)/self.nb_samples) + state_msg.imu_state.gyroscope[0]/self.nb_samples
        self.res_gyro[1] = self.res_gyro[1]*((self.nb_samples-1)/self.nb_samples) + state_msg.imu_state.gyroscope[1]/self.nb_samples
        self.res_gyro[2] = self.res_gyro[2]*((self.nb_samples-1)/self.nb_samples) + state_msg.imu_state.gyroscope[2]/self.nb_samples

        # Norm on all 3 axis ===================================================
        norm = math.sqrt( state_msg.imu_state.accelerometer[0]*state_msg.imu_state.accelerometer[0] + state_msg.imu_state.accelerometer[1]*state_msg.imu_state.accelerometer[1] + state_msg.imu_state.accelerometer[2]*state_msg.imu_state.accelerometer[2] )
        self.res_norm = self.res_norm*((self.nb_samples-1)/self.nb_samples) + norm/self.nb_samples

        print("Number of samples :", self.nb_samples)
        # print("Accelerometer biais :\n", self.res_acc)
        # print("Gyroscope bias :\n", self.res_gyro)
        print("Instantaneous accel norm : \n", norm)
        print("Mean of norm : \n", self.res_norm)
        print("-----------------------------------------")



def main(args=None):
    rclpy.init(args=args)

    bias_calc = Bias_calc()

    rclpy.spin(bias_calc)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    bias_calc.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
