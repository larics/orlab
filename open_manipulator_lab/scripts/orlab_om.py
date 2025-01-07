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

        # Define services  --- Comment this if working offline, without robot
        self.open_manipulator_send_command_service = rospy.ServiceProxy('/open_manipulator/goal_joint_space_path', SetJointPosition)

        # Variables
        self._q = [0, 0, 0, 0]

        # Parameters
        self.l = [0.077, 0.128, 0.024, 0.124, 0.044, 0.105]
        self.gamma = atan2(self.l[1], self.l[2])
        self.h = (self.l[1]**2 + self.l[2]**2)**0.5


    # LAB 1
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

        # Get q3
        star_1 = w[0]*cos(s_q1) + w[1]*sin(s_q1)     - self.l[5]*(w[3]*cos(s_q1) + w[4]*sin(s_q1)) + self.l[4]*w[5]
        star_2 = -(w[2] - self.l[0] - self.l[5]*w[5] - self.l[4]*(w[3]*cos(s_q1) + w[4]*sin(s_q1)) )
        help_q3 = acos( (1/(2*self.l[3]*self.h) )*( ( star_1 )**2 + ( star_2 )**2 - self.l[3]**2 - self.h**2))     

        s_q3_1 = -self.gamma + help_q3
        s_q3_2 = -self.gamma - help_q3
        
        # Get q2
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

        # Get q4
        help_q4 = atan2(-(w[3]*cos(s_q1) + w[4]*sin(s_q1)), -w[5])

        s_q4_1 = -s_q2_1 - s_q3_1 + help_q4
        s_q4_2 = -s_q2_2 - s_q3_2 + help_q4

        s_all = [[self.wrap2PI(s_q1), self.wrap2PI(s_q2_1), self.wrap2PI(s_q3_1), self.wrap2PI(s_q4_1)], 
                 [self.wrap2PI(s_q1), self.wrap2PI(s_q2_2), self.wrap2PI(s_q3_2), self.wrap2PI(s_q4_2)]]
        
        return s_all

    def get_closest(self, q_all, q0):
        # Find closest IK solution to robot pose q0
        # INPUT (1): all IK sollutions, 6xN
        # INPUT (2): Current joint state configuration
        # OUTPUT: q 6x1
        pass

    def wrap2PI(self, x):
        return (x-2*pi*floor(x/(2*pi)+0.5))
    

    # LAB 2
    def taylor_path(self, w_1, w_2, q_0, tol=0.01):
        # Funkcija implementira Taylorov postupak
        # ULAZI Funkcije:
        #   - w_1: Kartezijska poza pocetne tocke, izrazena kao numpy vektor 6x1
        #   - w_2: Kartezijska poza krajnje tocke, izrazena kao numpy vektor 6x1
        #   - q_0: Pocetna poza zglobova robota, izrazena kao numpy vektor Nx1 
        #   - tol: Zadana tolerancija, izrazena kao Float64
        # IZLAZI Funkcije: Tocke putanje u prostoru zglobova, izrazene kao numpy matrica, 
        #                  gdje je svaki novi red nova tocka.

        # Odredi inverz
        
        # Odredi međutočke u prostoru zglobova i prostoru alata

        # Odredi odstupanje
        
        # Provjeri jel uvjet zadovoljen
        #   JE -> Vrati rekurziju
        #   NIJE -> Rekurzivno pozovi funkciju taylor_path, idi lijevo i desno, spoji rezultate.

        pass

    def interpolate_q(self, Q, T_param):
        # Polinomska interpolacija jedinstvenim polinomom u prostoru zglobova.
        # Svaki od 4 reda ulazne matrice Q predstavlja vektor vrijednosti zglobova
        # kroz koji manipulator mora proci. Izlaz funkcije su vrijednosti 
        # otipkanog polinoma frekvencijom 10 Hz.
        # ULAZI Funkcije:
        #   - Q: Točke putanje u prostoru zglobova, izrazena kao numpy matrica Nx6
        #   - T_param: Parametričko vrijeme segmenta
        # IZLAZI Funkcije: Matrica točaka zglobova, otipkanog polinoma 
        #       frekvencijom 10 Hz.

        pass 

    def ho_cook(self, Q, v_max_lim, a_max_lim, f_s=250):
        # Funkcija implementira Ho-Cookovu metodu planiranja trajektorije
        # robotske ruke postivajuci zadana ogranicenja brzine (v_max) i akceleracije
        # (a_max). Funkcija kao rezultat vraca otipkanu trajektoriju frekvencijom
        # f_s u prostoru zglobova
        # ULAZI Funkcije:
        #   - Q: Točke putanje u prostoru zglobova, izrazena kao numpy matrica Nx6
        #   - v_max: Ograničenja brzina zglobova, izrazeno kao vektor 6x1
        #   - a_max: Ograničenja akceleracija zglobova, izrazeno kao vektor 6x1
        #   - f_s: Frekvencija otipkavanja trajektorije
        # IZLAZI Funkcije: Otipkane točke trajektorije, izrazene kao numpy matrica, 
        #                  gdje je svaki novi red nova tocka.

        m, n = np.shape(Q)

        # Racunanje parametrickog vremena (5.24)
        T = np.zeros((m, 1))

        # Prva iteracija
        iter_max = 10;
        iter_cnt = 0;
        S = 0

        # Postavi uvjet za loop petlju dok Ho-Cook nije zadovoljen

            # Izracunaj matrie M, A i Dq (5.50)
            # Popuni matricu M

            # Popuni matricu A

            # Izracunaj Dq


            # Izracunaj matricu B

            # Prvi segment (5.35)

            # Medjusegmenti (5.23)


            # Zadnji segment (5.41)

            # Odredi max. brzine i akceleracije


            # Odredi parametre uvjeta Sv i Sa



        # Otipkavanje trajektorije
        Ts = 1/f_s 
        # Prvi segment

        # Medjusegmenti segment

        # Zadnji segment
        
        # Spoji segmente i vrati rezultat u obliku
        #return [Q_q, Q_dq, Q_ddq]
        pass 

    # GENERAL FUNCTIONS
    def moveRobot(self, q, t):

        serviceReq = SetJointPositionRequest()
        serviceReq.joint_position.joint_name.append('joint1')
        serviceReq.joint_position.joint_name.append('joint2')
        serviceReq.joint_position.joint_name.append('joint3')
        serviceReq.joint_position.joint_name.append('joint4')
        #serviceReq.joint_position.joint_name.append('gripper')
        #serviceReq.joint_position.joint_name.append('gripper_sub')
        
        '''
        serviceReq.joint_position.joint_name.append('joint1')
        serviceReq.joint_position.joint_name.append('joint2')
        serviceReq.joint_position.joint_name.append('joint3')
        serviceReq.joint_position.joint_name.append('joint4')
        serviceReq.joint_position.position = [q[0], q[1], q[2], q[3]]

        serviceReq.path_time = t

        self.open_manipulator_send_command_service.call(serviceReq)
        '''
        for i in range(0, np.size(q, 0)):
            
            serviceReq.joint_position.position = [q[i, 0], q[i, 1], q[i, 2], q[i, 3]]

            serviceReq.path_time = t
            print (serviceReq)

            self.open_manipulator_send_command_service.call(serviceReq)
            rospy.sleep(t)

if __name__ == '__main__':

    rospy.init_node('ROBLAB_open_manipulator')
    node = ORLAB_OpenManipulator()

    # LAB 1
    # Test DK
    sol_dk = node.get_dk([-pi/6, -pi/5, pi/6, -pi/5])
    print ('DK: ', sol_dk)
    
    # Test IK
    sol_ik = node.get_ik(sol_dk)
    print('IK: ', sol_ik)

    # LAB 2
    # Test Taylor

    # Test interpolate

    # Test Ho-Cook
    
    # Move robot
    print(node.moveRobot(sol_ik[0], 4.0))