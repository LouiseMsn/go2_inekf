#!/bin/env python3

import rclpy
from rclpy.node import Node

# from nav_msgs.msg import Odometry
from unitree_go.msg import LowState
# from tf2_ros import TransformBroadcaster
# from geometry_msgs.msg import TransformStamped

import numpy as np

# ==============================================================================
# Main Class
# ==============================================================================

class Fake_IMU(Node):

    def __init__(self):
        super().__init__('fake_imu')
        
        self.publisher_ = self.create_publisher(LowState, '/fake_lowstate',10)

        timer_period = 0.01 #s
        self.timer_ = self.create_timer(timer_period, self.timer_callback)

    # ==========================================================================
    # Change parameters here
    # ==========================================================================
    # Accel parameters =========================================================
        self.a_bias = np.zeros((3,1)) # biais vector (imu frame)
        self.a_R = np.zeros((3,3)) # rotation between global frame and imu frame
        self.a_g = np.array([0,0,9.81]) # gravity vector (global frame)
        self.a_noise = np.zeros((3,1)) # noise vector (imu frame)

    # Gyro parameters ==========================================================
        self.g_bias = np.zeros((3,1))
        self.g_noise = np.zeros((3,1))
        self.fake_accel = np.zeros((3,1))
        self.fake_gyro = np.zeros((3,1))

    # Accel parameters =========================================================
        self.a_bias = np.zeros((3,1)) # biais vector (imu frame)
        self.a_R = np.zeros((3,3)) # rotation between global frame and imu frame
        self.a_g = np.array([[0],[0],[9.81]]) # gravity vector (global frame)
        self.a_noise = np.zeros((3,1)) # noise vector (imu frame)

        # Noise 
        #  to add offset change a_bias and not a_mean
        self.a_mean = [1,0,0] # x ,y, z
        self.a_standard_dev = [0,0,0]

        # Bias
        self.a_bias_mean = [0, 0, 0]
        self.a_bias_standard_dev = [0, 0, 0]

    # Gyro parameters ==========================================================
        self.g_bias = np.zeros((3,1))
        self.g_noise = np.zeros((3,1))

        # Noise 
        #  to add offset change a_bias and not a_mean
        self.g_mean = [0, 0, 0] #
        self.g_standard_dev = [0, 0, 0]
        
        # Bias
        self.g_bias_mean = [0, 0, 0]
        self.g_bias_standard_dev = [0, 0, 0]

    def calc_imu_data(self):
        # Compute noise & bias for all 3 axis of the accelerometer & gyrometer
        for i in range(0,3):
            self.a_noise[i] = np.random.normal(loc=self.a_mean[i],scale=self.a_standard_dev[i])
            self.a_bias[i]  = np.random.normal(loc=self.a_bias_mean[i], scale=self.a_bias_standard_dev[i])

            self.g_noise[i] = np.random.normal(loc=self.g_mean[i],scale=self.g_standard_dev[i])
            self.g_bias[i]  = np.random.normal(loc=self.g_bias_mean[i], scale=self.g_bias_standard_dev[i])

        #! only constant acceleration on X, everything else is null
        self.fake_accel = np.array([[1], [0], [0]]) + self.a_bias - np.matmul(self.a_R,self.a_g) + self.a_noise
        self.fake_gyro = np.array([[0], [0], [0]]) + self.g_bias + self.g_noise


        print("accel:\n",self.fake_accel)
        print("gyro:\n",self.fake_gyro)
            
    def timer_callback(self):
        fake_msg = LowState()

        self.calc_imu_data()

        for i in range (0,3):
            fake_msg.imu_state.accelerometer[i] = self.fake_accel[i][0]
            fake_msg.imu_state.gyroscope[i] = self.fake_gyro[i][0]

        self.publisher_.publish(fake_msg)
        



def main(args=None):
    rclpy.init(args=args)

    fake_imu = Fake_IMU()

    rclpy.spin(fake_imu)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    fake_imu.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()













"""
#!/bin/env python3

import rclpy
from rclpy.node import Node

from unitree_go.msg import LowState

import numpy as np

# ==============================================================================
# Main class
# ==============================================================================
 
class FakeIMU(Node):
    def __init__(self):
        # super().__init__('fake_imu')
        # self.publisher_ = self.create_publisher(LowState, '/fake_lowstate',10)

        # timer_period = 0.01 #s
        # self.timer_ = self.create_timer(timer_period,self.timer_callback())

        # # Accel parameters =====================================================
        # self.a_bias = np.zeros((3,1)) # biais vector (imu frame)
        # self.a_R = np.zeros((3,3)) # rotation between global frame and imu frame
        # self.a_g = np.array([0,0,9.81]) # gravity vector (global frame)
        # self.a_noise = np.zeros((3,1)) # noise vector (imu frame)
        
        # # Gyro parameters ======================================================
        # self.g_bias = np.zeros((3,1))
        # self.g_noise = np.zeros((3,1))
        print('yo')


    def timer_callback(self):
        fake_msg = LowState()



        for i in range (0,3):
            fake_msg.imu_state.accelerometer[i] = self.fake_accel[i][0]
            fake_msg.imu_state.gyroscope[i] = self.fake_gyro[i][0]

        self.publisher_.publish(fake_msg)
        

    def calc_imu_data(self):

        #! only constant acceleration on X ,everything else is null
        self.fake_accel = np.array([1, 0, 0]) + self.a_bias - np.matmul(self.a_R,self.a_g) + self.a_noise
        self.fake_gyro = np.array([0, 0, 0]) + self.g_bias + self.g_noise
    
# # Attributes ===================================================================
#     fake_accel = np.zeros((3,1))
#     fake_gyro = np.zeros((3,1))
#     # Accel parameters =====================================================
#     a_bias = np.zeros((3,1)) # biais vector (imu frame)
#     a_R = np.zeros((3,3)) # rotation between global frame and imu frame
#     a_g = np.array([0,0,9.81]) # gravity vector (global frame)
#     a_noise = np.zeros((3,1)) # noise vector (imu frame)
    
#     # Gyro parameters ======================================================
#     g_bias = np.zeros((3,1))
#     g_noise = np.zeros((3,1))



def main(args=None):
    rclpy.init(args=args)

    fake_imu = FakeIMU()

    rclpy.spin(fake_imu)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    fake_imu.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


    """