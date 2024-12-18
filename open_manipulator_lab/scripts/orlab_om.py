#!/usr/bin/env python

import sys
import csv
import copy
import rospy
import geometry_msgs.msg
import datetime
from math import pi, floor
from math import sin, cos
from math import sqrt
from math import atan2, asin, acos
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseArray
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tf

from numpy import genfromtxt

from open_manipulator_msgs.srv import SetJointPosition, SetJointPositionRequest
#import pandas as pd

class ORLAB_OpenManipulator():
    def __init__(self):
        
        # Define publishers
        self.file_PTP = '/home/kukapc/kuka_ws/src/OR_lab2_kuka/roblab_kuka/scripts/lab{NAD}_ptp.txt'

        # Define subscribers

        # Define services
        self.open_manipulator_send_command_service = rospy.ServiceProxy('/open_manipulator/goal_joint_space_path', SetJointPosition)

        # Variables
        self._q = [0, 0, 0, 0]

        # Parameters
        self.l = [0.077, 0.128, 0.024, 0.124, 0.044, 0.105]
        self.gamma = atan2(self.l[1], self.l[2])
        self.h = (self.l[1]**2 + self.l[2]**2)**0.5


    def get_dk(self, q):
        # Implement here direct kinematics
        # INPUT: q as a vector 4x1
        # OUTPUT: w as a vector 6x1

        print ('Enter DK - q: ', q)

        self.DH_params =  [[q[0],               self.l[0],  0,          -pi/2], 
                           [q[1]-self.gamma,    0,          self.h,     0],
                           [q[2]+self.gamma,    0,          self.l[3],  0],
                           [q[3],               0,          self.l[4],  -pi/2],
                           [0,                  self.l[5],  0,          0]]

        T_t = np.eye(4)

        for i in range(0, 5):
            T_temp = np.eye(4)
            T_temp[0][0] =  cos(self.DH_params[i][0])
            T_temp[0][1] = -cos(self.DH_params[i][3])*sin(self.DH_params[i][0])
            T_temp[0][2] =  sin(self.DH_params[i][3])*sin(self.DH_params[i][0])
            T_temp[0][3] =  self.DH_params[i][2]*cos(self.DH_params[i][0])
            T_temp[1][0] =  sin(self.DH_params[i][0])
            T_temp[1][1] =  cos(self.DH_params[i][3])*cos(self.DH_params[i][0])
            T_temp[1][2] = -sin(self.DH_params[i][3])*cos(self.DH_params[i][0])
            T_temp[1][3] =  self.DH_params[i][2]*sin(self.DH_params[i][0])
            T_temp[2][1] =  sin(self.DH_params[i][3])
            T_temp[2][2] =  cos(self.DH_params[i][3])
            T_temp[2][3] =  self.DH_params[i][1]

            #print (T_temp)

            T_t = copy.deepcopy(np.matmul(T_t, T_temp))

        #print (self.T_t)

        w = np.zeros(6)
        w[0] = T_t[0][3]
        w[1] = T_t[1][3]
        w[2] = T_t[2][3]
        w[3] = T_t[0][2]
        w[4] = T_t[1][2]
        w[5] = T_t[2][2]

        return w

    def get_ik(self, w):
        # Implement here inverse kinematics
        # INPUT (1): w 6x1 as a tool configuration vector, w = [x, y, z, wx, wy, wz]
        # INPUT (2): q0 4x1 as a joint_state vector of temp robot's position
        # OUTPU: q 4x1 as a closest feasible inverse solution to q0


        # Get q1
        s_q1 = atan2(w[1] , w[0] )
        #print ('Help q1: ', s_q1)

        # Get q3
        star_1 = w[0]*cos(s_q1) + w[1]*sin(s_q1)     - self.l[5]*(w[3]*cos(s_q1) + w[4]*sin(s_q1)) + self.l[4]*w[5]
        star_2 = -(w[2] - self.l[0] - self.l[5]*w[5] - self.l[4]*(w[3]*cos(s_q1) + w[4]*sin(s_q1)) )
        help_q3 = acos( (1/(2*self.l[3]*self.h) )*( ( star_1 )**2 + ( star_2 )**2 - self.l[3]**2 - self.h**2))     

        s_q3_1 = -self.gamma + help_q3
        s_q3_2 = -self.gamma - help_q3
        
        # Get q2 - PROVJERI
        help_q2_a1 = star_1
        help_q2_b1_1 = self.l[3]*cos(s_q3_1) + self.h*cos(self.gamma)
        help_q2_c1_1 = self.l[3]*sin(s_q3_1) - self.h*sin(self.gamma)
        help_q2_b1_2 = self.l[3]*cos(s_q3_2) + self.h*cos(self.gamma)
        help_q2_c1_2 = self.l[3]*sin(s_q3_2) - self.h*sin(self.gamma)
        help_q2_a2 = star_2
        help_q2_b2_1 = self.l[3]*sin(s_q3_1) - self.h*sin(self.gamma)
        help_q2_c2_1 = self.l[3]*cos(s_q3_1) + self.h*cos(self.gamma)
        help_q2_b2_2 = self.l[3]*sin(s_q3_2) - self.h*sin(self.gamma)
        help_q2_c2_2 = self.l[3]*cos(s_q3_2) + self.h*cos(self.gamma)

        

        s_q2_1 = atan2(help_q2_a2*help_q2_b1_1 - help_q2_a1*help_q2_b2_1, help_q2_a2*help_q2_c1_1 + help_q2_a1*help_q2_c2_1)
        s_q2_2 = atan2(help_q2_a2*help_q2_b1_2 - help_q2_a1*help_q2_b2_2, help_q2_a2*help_q2_c1_2 + help_q2_a1*help_q2_c2_2)
        #print ('Help q2: ', self.wrap2PI(s_q2_1), self.wrap2PI(s_q2_2))

        # Get q4
        help_q4 = atan2(-(w[3]*cos(s_q1) + w[4]*sin(s_q1)), -w[5])

        s_q4_1 = -s_q2_1 - s_q3_1 + help_q4
        s_q4_2 = -s_q2_2 - s_q3_2 + help_q4

        #print( 'Help q4: ', s_q4_1, s_q4_2)
        s_all = [[self.wrap2PI(s_q1), self.wrap2PI(s_q2_1), self.wrap2PI(s_q3_1), self.wrap2PI(s_q4_1)], 
                 [self.wrap2PI(s_q1), self.wrap2PI(s_q2_2), self.wrap2PI(s_q3_2), self.wrap2PI(s_q4_2)]]

        #print ('Check 1: ', self.get_dk(s_all[0]))
        #print ('Check 2: ', self.get_dk(s_all[1]))
        #print ('Sollution: ', s_all)
        
        return s_all

    def get_closest(self, q_all, q0):
        # Find closest IK solution to robot pose q0
        # INPUT (1): all IK sollutions, 6xN
        # INPUT (2): Current joint state configuration
        # OUTPUT: q 6x1
        pass

    def wrap2PI(self, x):
        return (x-2*pi*floor(x/(2*pi)+0.5))

    def moveRobot(self, q, t):

        serviceReq = SetJointPositionRequest()

        print (serviceReq)

        serviceReq.joint_position.joint_name.append('joint1')
        serviceReq.joint_position.joint_name.append('joint2')
        serviceReq.joint_position.joint_name.append('joint3')
        serviceReq.joint_position.joint_name.append('joint4')
        #serviceReq.joint_position.joint_name.append('gripper')
        #serviceReq.joint_position.joint_name.append('gripper_sub')
        serviceReq.joint_position.position = [q[0], q[1], q[2], q[3]]

        serviceReq.path_time = t

        self.open_manipulator_send_command_service.call(serviceReq)

if __name__ == '__main__':

    rospy.init_node('ROBLAB_open_manipulator')
    node = ORLAB_OpenManipulator()

    
    sol_dk = node.get_dk([-pi/6, -pi/5, pi/6, -pi/5])
    print ('DK: ', sol_dk)
    sol_ik = node.get_ik(sol_dk)
    print('IK: ', sol_ik)

    #sol_ik = node.get_ik([-0.15, -0.01, 0.1, 0, 0, -1])
    #print('IK: ', sol_ik)

    #print ('DK1: ', node.get_dk(sol_ik[0]))
    #print ('DK2: ', node.get_dk(sol_ik[1]))
    
    print(node.moveRobot(sol_ik[0], 4.0))
    #print(node.moveRobot([0, 0, 0, 0], 4.0))