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
        
        
        # Variables
        self._q = [0, 0, 0, 0]

    def get_dk(self, q):
        # Implement here direct kinematics
        # INPUT: q as a vector 4x1
        # OUTPUT: w as a vector 6x1

        # TODO:


        # OUTPUT:
        w = np.zeros(6)
        #w[0] = ...
        #w[1] = ...
        #w[2] = ...
        #w[3] = ...
        #w[4] = ...
        #w[5] = ...

        return w

    def get_ik(self, w):
        # Implement here inverse kinematics
        # INPUT (1): w 6x1 as a tool configuration vector, w = [x, y, z, wx, wy, wz]
        # OUTPU: q 4xN as all inverse solutions

        # TODO

        # Output
        s_all = []
        return s_all

    def get_closest(self, q_all, q0):
        # Find closest IK solution to robot pose q0
        # INPUT (1): all IK sollutions, 6xN
        # INPUT (2): Current joint state configuration
        # OUTPUT: q 6x1

        # TODO:

        # Output
        q = np.zeros(6)
        #q[0] = ..
        #q[1] = ..
        #q[2] = ..
        #q[3] = ..
        #q[4] = ..
        #q[5] = ..
        return q

    def wrap2PI(self, x):
        return (x-2*pi*floor(x/(2*pi)+0.5))

if __name__ == '__main__':

    rospy.init_node('ROBLAB_open_manipulator')
    node = ORLAB_OpenManipulator()

    
    sol_dk = node.get_dk([0, 0, 0, 0])
    print ('DK: ', sol_dk)
    
    sol_ik = node.get_ik(sol_dk)
    #print('IK: ', sol_ik)
