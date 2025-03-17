#!/bin/env python3

import math
import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from unitree_go.msg import LowState
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


import pdb;pdb.set_trace()

# TODO from RobotState import *

# ==============================================================================
# Main Class
# ==============================================================================
class Inekf(Node):

    def __init__(self):
        super().__init__('inekf')
        print("==GO2 InEKF launched==")
        # ROS2 =================================================================
        self.declare_parameter("base_frame", "base")
        self.declare_parameter("odom_frame", "odom")

        self.vel_publisher_ = self.create_publisher(Odometry, 'odometry/feet_vel', 10)
        self.pos_publisher_ = self.create_publisher(Odometry, 'odometry/feet_pos', 10)
        self.subscription_ = self.create_subscription(LowState,
                                                    '/lowstate',
                                                    self.listener_callback,
                                                    10)
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Filter and global variables ==========================================
        self.g_ = np.array(([0],[0],[-9.81])) # Gravity vector #! changed /!\
        
        self.DT_MIN = 1e-6
        self.DT_MAX = 1
        self.dt = 0

        # Init Robot state =====================================================
        # initiated by constructor in OG class
        self.state_ = RobotState()

        R0 = np.identity(3) # TODO: verify the imu frame is in the same orientation as the state frame
        v0 = np.zeros((3,1))
        p0 = np.zeros((3,1))
        bg0 = np.zeros((3,1)) # bg = Bias Gyroscope
        ba0 = np.zeros((3,1)) # ba = Bias Accelerometer

        self.state_.setRotation(R0)
        self.state_.setVelocity(v0)
        self.state_.setPosition(p0)
        self.state_.setGyroscopeBias(bg0)
        self.state_.setAccelerometerBias(ba0)

        # Init state noise =====================================================
        # Q = covariance matrix
        
        """     
        #* somewhat real parameters
        self.noise_params = NoiseParam()
        self.noise_params.setGyroscopeNoise(math.sqrt(1e-6)) # sqrt(1e-6)
        self.noise_params.setAccelerometerNoise(math.sqrt(6e-2))
        gyro_bias = np.diag([-0.001,-0.0007,-0.005])
        self.noise_params.setGyroscopeBiasNoise(gyro_bias)
        accel_bias= np.diag([-0.14,-0.07,0.23])
        self.noise_params.setAccelerometerBiasNoise(accel_bias)
        self.noise_params.setContactNoise(1e-10)
        """

        #* perfect parameters
        self.noise_params = NoiseParam()
        self.noise_params.setGyroscopeNoise(0)
        self.noise_params.setAccelerometerNoise(0)
        gyro_bias = np.diag([0,0,0])
        self.noise_params.setGyroscopeBiasNoise(gyro_bias)
        accel_bias= np.diag([0,0,0])
        self.noise_params.setAccelerometerBiasNoise(accel_bias)
        self.noise_params.setContactNoise(0)


        # Data from go2 ========================================================
        self.imu_measurement_ = np.zeros((6,1))
        self.imu_measurement_prev_ = np.zeros((6,1))
        self.feet_contacts_ = np.zeros((4))
        self.__contacts={0:False,
                        1:False,
                        2:False,
                        3:False}
        
        self.joints_unitree_2_urdf = [1,0,3,2] # index = urdf convention, value = unitree joint numbering
        
        self.__estimated_contact_positions_ = {0:0, #! not sure if correct
                                                1:0,
                                                2:0,
                                                3:0}
        
        # double = float in python
        self.t : float = 0
        self.t_prev : float = 0


    def listener_callback(self, state_msg):
        # TODO verify if timestamp can be used as time
        self.t = state_msg.tick * 0.001 # convert from ms to s like it seems to be in OG code
    
        # IMU measurement - used for propagation ===============================
        #! gyroscope meas in filter are radians
        self.imu_measurement_[0][0] = state_msg.imu_state.gyroscope[0]
        self.imu_measurement_[1][0] = state_msg.imu_state.gyroscope[1]
        self.imu_measurement_[2][0] = state_msg.imu_state.gyroscope[2]

        self.imu_measurement_[3][0] = state_msg.imu_state.accelerometer[0]
        self.imu_measurement_[4][0] = state_msg.imu_state.accelerometer[1]
        self.imu_measurement_[5][0] = state_msg.imu_state.accelerometer[2]

        # breakpoint()
        if(self.dt > self.DT_MIN and self.dt < self.DT_MAX):
            #propagate using previous measurement
            self.propagate()

        print("State:\n",
               "Position:\n", self.state_.getPosition(),
               "\nRotation:\n",self.state_.getRotation())


        # CONTACT data : order changed to match URDF #? not sure if needed =====
        for i in range (0,4):
            if(state_msg.foot_force[self.joints_unitree_2_urdf[i]]>20):
                self.__contacts[i] = True
            else:
                self.__contacts[i] = False

        # KINEMATIC data =======================================================
        quaternion = np.zeros((4))
        quaternion[0] = state_msg.imu_state.quaternion[0]
        quaternion[1] = state_msg.imu_state.quaternion[1]
        quaternion[2] = state_msg.imu_state.quaternion[2]
        quaternion[3] = state_msg.imu_state.quaternion[3]
        # TODO normalize quaternion && find quat type


        # TODO: add feet vel and feet pos (additionnal info on base)
        self.dt = self.t - self.t_prev
        self.t_prev = self.t
        self.imu_measurement_prev_ = self.imu_measurement_

    def propagate(self):
        # Angular Velocity
        w =  self.imu_measurement_prev_[:3] - self.state_.getGyroscopeBias() # first three values of imu (gyro) - gyroscope bias

        # Linear Acceleration
        # first three values of imu (accel) - accel bias
        a = self.imu_measurement_prev_[3:] - self.state_.getAccelerometerBias()
        
        X = self.state_.getX()
        P = self.state_.getP()

        # Extract State
        R = self.state_.getRotation()
        v = self.state_.getVelocity()
        p = self.state_.getPosition()

        # Strapdown IMU motion model
        phi = w*self.dt # vecteur (3,1)
        R_pred = np.matmul(R, self.Exp_SO3(phi)) # vecteur (3,3)
        v_pred = v + (np.matmul(R,a) - self.g_)*self.dt # vecteur (3,1)
        p_pred = p + v*self.dt + 0.5*(np.matmul(R,a) - self.g_)*self.dt*self.dt # vecteur (3,1)

        # Set new state (bias has constant dynamics)
        self.state_.setRotation(R_pred)
        self.state_.setVelocity(v_pred)
        self.state_.setPosition(p_pred)

        # Linearized invariant error dynamics ==============================
        dimX = self.state_.dimX()
        dimP = self.state_.dimP()
        dimTheta = self.state_.dimTheta()
        A = np.zeros((dimP,dimP)) #! find out what is A    # Eigen::MatrixXd A = Eigen::MatrixXd::Zero(dimP,dimP);

        # Inertial terms ===================================================
        A[3:3+3,0:0+3] = self.skew(self.g_) # TODO: Efficiency could be improved by not computing the constant terms every time
        A[6:6+3,3:3+3] = np.identity(3) 

        # Bias terms =======================================================
        A[0:0+3,dimP-dimTheta:dimP-dimTheta+3] = -R 
        A[3:3+3,dimP-dimTheta+3:dimP-dimTheta+3+3] = -R 
        
        for i in range(3,dimX):
            A[3*i-6 : 3*i-6+3, dimP-dimTheta : dimP-dimTheta+3] = np.matmul(-self.skew(X[0:0+3,i:i+1]),R)
  
        # Noise terms ======================================================
        Qk = np.zeros((dimP,dimP)) #* Landmark noise terms will remain 0
        
        Qk[0:0+3, 0:0+3] = self.noise_params.getGyroscopeCov()
        Qk[3:3+3, 3:3+3] = self.noise_params.getAccelerometerCov() 

        for key,value in self.__estimated_contact_positions_.items():
            Qk[3+3*(value-3):3+3*(value-3)+3, 3+3*(value-3):3+3*(value-3)+3] = self.noise_params.getContactCov() #! seems not exact

        Qk[dimP-dimTheta:dimP-dimTheta+3, dimP-dimTheta:dimP-dimTheta+3] = self.noise_params.getGyroscopeBiasCov() 
        Qk[dimP-dimTheta+3:dimP-dimTheta+3+3,dimP-dimTheta+3:dimP-dimTheta+3+3] = self.noise_params.getAccelerometerBiasCov()       

        # Discretization ===================================================
        I = np.identity(dimP) 
        Phi = I + A*self.dt # TODO: explore using the full exp() instead
        Adj = I 
        Adj[0:0+dimP-dimTheta,0:0+dimP-dimTheta] = self.Adjoint_SEK(X)  # Approx 200 microseconds
        PhiAdj = np.matmul(Phi , Adj) 
        Qk_hat = np.matmul(np.matmul( PhiAdj,Qk) , np.transpose(PhiAdj))*self.dt # Approximated discretized noise matrix (faster by 400 microseconds)

        # Propagate Covariance =============================================
        P_pred = np.matmul(np.matmul(Phi,P),np.transpose(Phi)) + Qk_hat 

        # Set new covariance
        self.state_.setP(P_pred)

        # self.state_.print()

        # Transform from odom_frame (unmoving) to base_frame (tied to robot base)
        timestamp = self.get_clock().now().to_msg()
        self.transform_msg.header.stamp = timestamp
        self.transform_msg.child_frame_id = self.get_parameter("base_frame").value
        self.transform_msg.header.frame_id = self.get_parameter("odom_frame").value

        # # translation + convert ?
        # self.state_.printPosition()

        # ROS2 transform
        self.transform_msg.transform.translation.x = self.state_.getPosition()[0][0]
        self.transform_msg.transform.translation.y = self.state_.getPosition()[1][0]
        self.transform_msg.transform.translation.z = self.state_.getPosition()[2][0]

        rotation_mat = self.state_.getRotation()
        r = Rotation.from_matrix([[rotation_mat[0][0], rotation_mat[0][1], rotation_mat[0][2]],
                                  [rotation_mat[1][0], rotation_mat[1][1], rotation_mat[1][2]],
                                  [rotation_mat[2][0], rotation_mat[2][1], rotation_mat[2][2]]])

        x,y,z,w = r.as_quat()
        print("RPY rotation:", r.as_euler("xyz",degrees=True))

        self.transform_msg.transform.rotation.x = x
        self.transform_msg.transform.rotation.y = y
        self.transform_msg.transform.rotation.z = z
        self.transform_msg.transform.rotation.w = w
        
        #! Rotation matrix had a det that is further and further from 1 TODO
        


        self.tf_broadcaster.sendTransform(self.transform_msg)
        

    def Exp_SO3(self,w):
        # Computes the vectorized exponential map for SO(3)
        TOLERANCE = 1e-10
        A = self.skew(w)
        theta = np.linalg.norm(w)
        
        if(theta < TOLERANCE):
            return np.identity(3)
        
        R = np.identity(3) + (math.sin(theta)/theta)*A + ((1-math.cos(theta))/(theta*theta))*np.matmul(A,A)
        return R
    
    def skew(self,vector):
        # convert vector (column) to skew-symmetric matrix
        M = np.zeros((3,3))
        M = np.array([[0,         -vector[2][0],vector[1][0]],
                    [vector[2][0], 0           ,-vector[0][0]],
                    [-vector[1][0],vector[0][0],   0]])
        return M

    def Adjoint_SEK(self, X):
        # Compute Adjoint(X) for X in SE_K(3)
        K = X.shape[1]-3;
        Adj = np.zeros((3+3*K,3+3*K)) # Eigen::MatrixXd Adj = Eigen::MatrixXd::Zero(3+3*K, 3+3*K);
        R = X[0:0+3,0:0+3] # Eigen::Matrix3d R = X.block<3,3>(0,0);
        Adj[0:0+3,0:0+3] = R # Adj.block<3,3>(0,0) = R;

        #!!
        for i in range(0,K):
            Adj[3+3*i:3+3*i+3, 3+3*i:3+3*i+3] = R #Adj.block<3,3>(3+3*i,3+3*i) = R;
            Adj[3+3*i:3+3*i+3, 0:0+3] = np.matmul(self.skew(X[0:0+3,3+i:3+i+1]),R) #Adj.block<3,3>(3+3*i,0) = skew(X.block<3,1>(0,3+i))*R;
        return Adj

    def Correct(self, observation):
        P = self.state_.getP()
        PHT = np.matmul(P,observation.H.transpose())
        S = np.matmul(observation.h , PHT) + observation.N
        K = np.matmul(PHT, S.linalg.inv())

        # Copy X along the diagonals if more than one measurement
        BigX = np.empty((0,0))
        rows, columns = observation.Y.shape
        self.state_.copyDiag(rows/self.state_.dimX(), BigX)

        # Compute correction terms
        Z = np.matmul(BigX, observation.Y) - observation.b
        delta = np.matmul(np.matmul( K, observation.PI),Z)
        delta_rows = delta.shape[0]
        dX = self.Exp_SEK3(delta[0:0 + delta_rows - self.state_.dimTheta()])
        dTheta = delta[delta_rows - self.state_.dimTheta() : delta_rows]

        # Update state
        X_new = np.matmul(dX , self.state_.getX())
        Theta_new = self.state_.getTheta() + dTheta
        self.state_.setX(X_new)
        self.state_.setTheta(Theta_new)

        # Update Covariance
        IKH = np.identity(self.state_.dimP()) - np.matmul(K, observation.H)
        P_new = np.matmul(np.matmul(IKH, P), IKH.transpose()) + np.matmaul(np.matmul(K, observation.N), K.transpose())

        self.state_.setP(P_new)

    def CorrectKinematics(self, measured_kinematics): #meas_kin = const vectorkinematics&
            
        # ! start of OG code =======================================================
            # Eigen::VectorXd Y;
            # Eigen::VectorXd b;
            # Eigen::MatrixXd H;
            # Eigen::MatrixXd N;
            # Eigen::MatrixXd PI;


        Y = np.empty((0,0)) # vecteur vertical
        b = np.empty((0,0)) # vecteur vertical
        H = np.empty((0,0)) # matrice
        N = np.empty((0,0)) # matrice
        PI = np.empty((0,0))# matrice

        R = self.state_.getRotation() # Eigen::Matrix3d R = state_.getRotation();
        remove_contacts = np.empty((0,0)) 
        new_contacts = np.empty((0,0)) #* vector replaced by numpy arrays
        used_contact_ids = np.empty((0,0))
        it_estimated = None

        # IDs = foot ID
        for i in measured_kinematics:
            # Detect and skip if an ID is not unique (this would cause singularity issues in InEKF::Correct)
            if i.id in used_contact_ids:
                continue
            else:
                used_contact_ids = np.append(used_contact_ids, i.id) #* le marquer comme étant "traité"

            # find the foot ID in the contacts_ vector (attibute of inekf class)
            if(i.id in self.contacts_) :
                contact_indicated = self.contacts_[i.id] # bool value of the contact = 1 or 0
            else:
                continue # skip if contact state is unknown

            # See if we can find an estimated_contact_positions for the foot ID
            # #* est ce que la position du contact entrain d'être traité a déjà été calculée?
            if(i.id in self.__estimated_contact_positions_):
                it_estimated = (i.id,self.__estimated_contact_positions[i.id]) 
                found = True
            else:
                found = False


            if(not contact_indicated and found):
                # if no contact for this foot and the foot ID was found in contacts estimated
                remove_contacts = np.append(remove_contacts, it_estimated) # add foot ID to the remove list

            elif(contact_indicated and not found):
                #  If contact is indicated and foot id id is not found in estimated_contacts_, then augment state
                new_contacts = np.append(new_contacts, i)

            elif(contact_indicated and found):
                dimX = self.state_.dimX()
                dimP = self.state_.dimP()
                startIndex = 0

                # Fill out Y
                rows = Y.shape[0]
                startIndex =  int(rows) #? can be replaced with Y.shape[0] but less readable

                if rows == 0:
                    Y = np.zeros((dimX,1)) # purely because pad does not work with empty arrays
                else: 
                    Y = np.pad(Y,[(0,dimX),(0,0)], mode='constant', constant_values=0)

                Y[startIndex:startIndex+3] = i.pose[0:0+3,3:3+1]
                Y[startIndex+4][0] = 1
                Y[startIndex+it_estimated[1]]= -1 #! did not understand

                # Fill out b
                rows = b.shape[0]
                startIndex =  int(rows)
                if rows == 0:
                    b = np.zeros((dimX,1)) # purely because pad does not work with empty arrays
                else: 
                    b = np.pad(b,[(0,dimX),(0,0)], mode='constant', constant_values=0)

                b[startIndex+4][0] = 1
                b[startIndex+it_estimated[1]]= -1

                # Fill out H
                rows, columns = H.shape
                startIndex = int(rows)
                if rows == 0:
                    H = np.zeros((3,dimP))
                else:
                    H = np.pad(H,[(0,3),(0,dimP)], mode='constant', constant_values=0)

                H[startIndex:startIndex+3,0:0+dimP] = np.zeros((3,dimP))
                H[startIndex:startIndex+3,6:6+3] = np.identity(3)
                H[startIndex:startIndex+3,3*it_estimated[1]-6:3*it_estimated[1]-6+3] = - np.identity(3)
                
                # Fill out N
                rows,columns = N.shape
                if rows == 0:
                    N = np.zeros((3,3))
                else:
                    N = np.pad(N,[(0,3),(0,3)], mode='constant', constant_values=0)
                
                N[startIndex:startIndex+3,0:0+startIndex] = np.zeros((3,startIndex))
                N[0:0+startIndex:startIndex,startIndex+3] = np.zeros((startIndex,3))
                N[startIndex:startIndex+N,startIndex:startIndex+N] = np.matmul(np.matmul(R,i.covarience[3:3+3,3:3+3]),R.transpose())

                # FIll out PI
                rows, columns = PI.shape
                startIndex = int(rows)
                startIndex2 = int(columns)
                if startIndex == 0:
                    PI = np.zeros((3,dimX))
                else:
                    PI = np.pad(N,[(0,3),(0,dimX)], mode='constant', constant_values=0)

                PI[startIndex:startIndex+3,0:0+startIndex2] = np.zeros((3,startIndex2))
                PI[0:0+startIndex,startIndex2:startIndex2+dimX] = np.zeros((startIndex,dimX))
                PI[startIndex:startIndex+3,startIndex2:startIndex2+dimX] = np.zeros((3,dimX))
                PI[startIndex:startIndex+3,startIndex2:startIndex2+3] = np.identity(3)

            else:
                continue
#? tag d'avancement
                
            obs = Observation(Y,b,H,N,PI)
            if not obs.empty():
                self.Correct(obs) # TODO add correct function
            
            if remove_contacts.size > 0:
                X_rem = self.state_.getX()
                P_rem = self.state_.getP()
                for iterator in remove_contacts:
                    self(self.estimated_contact_positions_[iterator[0]])


    # Attributes ===============================================================
    transform_msg = TransformStamped()

    estimated_landmarks_ = {} # int:int
    contacts_ = {} # int:bool
    estimated_contact_positions_ = {} # int:int


# ==============================================================================
# RobotState Class
# ==============================================================================
class RobotState():
    def __init__(self):
        # TODO add other constructors like in OG code
        self.__X = np.identity(5)
        self.__Theta = np.zeros((6,1))
        self.__P = np.identity(15)
    # Getters Setters ===========================================================
    def getX(self):
        return self.__X
    
    def setX(self,X):
        self.__X = X
    
    def getTheta(self):
        return self.__Theta
    
    def setTheta(self, Theta):
        self.__Theta = Theta
    
    def getP(self):
        return self.__P
    
    def setP(self,P):
        self.__P = P

    # ROTATION : 3x3 matrix starting in (0,0)
    def getRotation(self):
        return self.__X[0:3,0:3]
    
    def setRotation(self, rot):
        self.__X[0:3,0:3] = rot

    # VELOCITY : 3x1 vector starting in (0,3) 
    def getVelocity(self):
        return self.__X[0:3,3:4]
    
    def setVelocity(self,vel):
        self.__X[0:3,3:4] = vel
    
    # POSITION : 3x1 vector starting in (0,4)
    def getPosition(self):
        return self.__X[0:3,4:5]
    
    def setPosition(self, pos):
        self.__X[0:3,4:5] = pos

    # ---
    def dimX(self):
        return self.__X.shape[1]
    
    def dimTheta(self):
        return self.__Theta.shape[0]
    
    def dimP(self):
        return self.__P.shape[1]
        
    def getGyroscopeBias(self):
        return self.__Theta[0:3,0:1]

    def setGyroscopeBias(self,bg):
        self.__Theta[0:3,0:1] = bg

    def getAccelerometerBias(self):
        return self.__Theta[3:6,0:1]

    def setAccelerometerBias(self,ba):
        self.__Theta[3:6,0:1] = ba

    def copyDiag(self, n, BigX):
        # Returns a (n*dimX,n*dimX) array where X is copied along the diagonal
        # TODO find more elegant way for returning BigX (->pointers if converted to C++ ('_'))
        dimX = self.dimX()
        
        for i in range(0,n+1):
            startIndex = BigX.shape[0]
            
            new_BigX = np.zeros((startIndex + dimX, startIndex + dimX))
            new_BigX[ : BigX.shape[0], : BigX.shape[1]] = BigX
           
            new_BigX[ startIndex : startIndex+dimX , startIndex : startIndex+dimX] = self.__X

            BigX=new_BigX
        return BigX
    
    def print(self):
        np.set_printoptions(precision=2)
        print("--------- Robot State -------------")
        print("X:\n",self.__X)
        print("Theta:\n",self.__Theta)
        print("P:",self.__P)
        print("-----------------------------------")

    def printPosition(self):
        np.set_printoptions(precision=2)
        print("--------- Robot Position -------------")
        print(self.getPosition())
        print("-----------------------------------")


# ==============================================================================
# NoiseParam Class
# ==============================================================================
class NoiseParam():
    def __init__(self):

        self.__Qg  = np.array((3,3))
        self.__Qa  = np.array((3,3))
        self.__Qbg = np.array((3,3))
        self.__Qba = np.array((3,3))
        self.__Ql  = np.array((3,3))
        self.__Qc  = np.array((3,3))

        self.setGyroscopeNoise(0.01)
        self.setAccelerometerNoise(0.1)
        self.setGyroscopeBiasNoise(np.identity(3)*0.01*0.01)
        self.setAccelerometerBiasNoise(np.identity (3)*0.01*0.01)
        # self.setLandmarkNoise(0.1) #* not implemented
        self.setContactNoise(0.1)
        

    # Getters Setters =========================================================

    def setGyroscopeNoise(self,value):
        self.__Qg = np.identity(3)*value*value

    def getGyroscopeCov(self):
        return self.__Qg
    
    def setAccelerometerNoise(self,value):
        self.__Qa = np.identity(3)*value*value
    
    def getAccelerometerCov(self):
        return self.__Qa
    
    def setGyroscopeBiasNoise(self,matrix):
        self.__Qbg = matrix
        # self.__Qbg = np.identity(3)*value*value

    def getGyroscopeBiasCov(self):
        return self.__Qbg
    
    def setAccelerometerBiasNoise(self,matrix):
        self.__Qba = matrix
        # self.__Qba = np.identity(3)*value*value

    def getAccelerometerBiasCov(self):
        return self.__Qba

    def setContactNoise(self,value):
        self.__Qc = np.identity(3)*value*value

    def getContactCov(self):
        return self.__Qc

    #* void setLandmarkNoise(double std) and  Eigen::Matrix3d getContactCov() not implemented because not used

# ==============================================================================
# Kinematics Class
# ==============================================================================
class Kinematics():
    def __init__(self):
        id = 0
        pose = np.zeros((4,4))
        cov = np.zeros((6,6))

# ==============================================================================
# Observation Class
# ==============================================================================
class Observation():
    def __init__(self,Y_val,b_val,H_val,N_val,PI_val):
        Y = Y_val
        b = b_val
        H = H_val
        N = N_val
        PI = PI_val

    def empty(self):
         rows, columns = self.Y.shape
         return int(rows) == 0

    def print(self):
        print("Y:\n",self.Y,"\n",
              "b:\n",self.b,"\n",
              "H:\n",self.H,"\n",
              "N:\n",self.n,"\n",
              "PI;\n",self.PI,"\n") #! potential bug due to name


def main(args=None):
    rclpy.init(args=args)

    inekf = Inekf()
    
    rclpy.spin(inekf)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    inekf.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
