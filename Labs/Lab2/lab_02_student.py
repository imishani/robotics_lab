import numpy as np
from sympy import *
from numpy import linalg as LA
from numpy.linalg import inv
import time
import logging
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.pyplot as plt
from matplotlib import animation
from scipy.spatial.transform import Rotation
import math
import os

if os.name == 'nt':
    import msvcrt
else:
    import tty, termios
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2
import sys
from Lab02_intro import *
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../Lab1/"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../common/robot"))
from robot_actions import *
import utilities


class robotic_arm():

    def __init__(self):
        self.joints = 0
        self.alpha = None
        self.a = None
        self.q = None
        self.d = None
        self.dh_params = {}
        self.tf_matrices_list = []


    def set_joints(self, joint_number):
        if joint_number > 0:
            self.joints = int(joint_number)
            self.set_dh_params()
        else:
            raise ('Joints Number has to be positive')

    def set_dh_params(self):
        self.alpha = symbols('alpha1:' + str(self.joints + 1))
        self.a = symbols('a1:' + str(self.joints + 1))
        self.q = symbols('q1:' + str(self.joints + 1))
        self.d = symbols('d1:' + str(self.joints + 1))

    def show_dh_params(self):
        print('DH Parameters are: {}'.format(self.dh_params))

    def set_dh_param_dict(self, dh_params_values):

        self.dh_params = dh_params_values
        self.set_tranform_matrices()

    def set_tranform_matrices(self):
        T_01 = dh(alpha1, a1, d1, q1).subs(self.dh_params)
        self.tf_matrices_list.append(T_01)
        T_12 = dh(alpha2, a2, d2, q2).subs(self.dh_params)
        self.tf_matrices_list.append(T_01 * T_12)
        T_23 = dh(alpha3, a3, d3, q3).subs(self.dh_params)
        self.tf_matrices_list.append(T_01 * T_12 * T_23)
        T_34 = dh(alpha4, a4, d4, q4).subs(self.dh_params)
        self.tf_matrices_list.append(T_01 * T_12 * T_23 * T_34)
        T_45 = dh(alpha5, a5, d5, q5).subs(self.dh_params)
        self.tf_matrices_list.append(T_01 * T_12 * T_23 * T_34 * T_45)
        T_56 = dh(alpha6, a6, d6, q6).subs(self.dh_params)
        T = T_01 * T_12 * T_23 * T_34 * T_45 * T_56
        self.tf_matrices_list.append(T)

    def show_transform_matrices(self):
        print('Transform Matrices are: {}'.format(self.tf_matrices_list))

    @staticmethod
    def TF_matrix(alpha, a, d, q):

    def forward_kinematics(self, theta_list):
        """
        calculate forward kinematic for a desired theta_list
        :param data: theta_list
        :type data: list
        :return: forward kinematics list SE-3  of the form [x y z roll pitch yaw] w.r.t base_link
        :rtype: list
        """

        theta_dict = {}
        # homogenous transformation matrix from base_link to end_effector [type: syms]
        T_0G = self.tf_matrices_list[-1]

        for i in range(len(theta_list)):
            theta_dict[self.q[i]] = theta_list[i]

        # homogenous transformation matrix from base_link to end_effector [type: numeric matrix]
        T_0G_eval = T_0G.evalf(subs=theta_dict, chop=True, maxn=4)

        # TODO: calculate [x,y,z,roll,pitch,yaw] ### change to tranformation matrix

        self.current_pos =  xyz_rpy(T_0G_eval)

        return self.current_pos

def move_to_angle_conf(Q):

    # Parse arguments
    args = utilities.parseConnectionArguments()

    # Create connection to the device and get the router
    with utilities.DeviceConnection.createTcpConnection(args) as router:
        # Create required services
        base = BaseClient(router)
        base_cyclic = BaseCyclicClient(router)

        input("Remove any objects near the arm and press Enter")
        # Example core
        success = True
        flag = True
        display = True

        while flag and success:

            if display:
                key = input("Press H to move the arm  to home position\n"
                      "Press A to move the arm to desired angular action: " + str(Q) + '\n'
                      "To Quit press Q")
                display = False

            if str(key) == 'h' or 'H':
                success &= example_move_to_home_position(base)
                if success:
                    print('Successfully moved to home position')
                    display = True
                else:
                    print('Huston, we have a problem, please call the instructor')

            if str(key) == 'A' or 'a':
                success &= example_angular_action_movement(base)
                if success:
                    print('Successfully moved to arm to desired angular action')
                    display = True
                else:
                    print('Huston, we have a problem, please call the instructor')


# if __name__ == '__main__':
def run():
    # For simplicity, we used symbolic function for differentiation
    # A tutorial can be found here: https://docs.sympy.org/latest/tutorial/index.html

    #######################
    ###### Part 1 #########
    #######################

    arm = robotic_arm()     # Initialize a generic robotic arm
    arm.set_joints(6)       # Define the amount of joints

    # DH params


    # TODO: fill the following dict as described in the lab notes

    arm.set_dh_param_dict(set_dh_table()) # set dh paramters

    # TODO: enter the desired joint state target configurations
    angle_conf_target_dict = angles_to_follow()
    angle_conf_eval = {}

    # TODO: fill the blank line in the forward_kinematics function
    for i in range(len(angle_conf_target_dict)):
        angle_conf_eval.update({'t'+ str(i + 1): arm.forward_kinematics(angle_conf_target_dict['t' + str(i + 1)])})

    # Import arm modules and move to each configuration
    for i in range(len(angle_conf_eval)):
        move_to_angle_conf(angle_conf_eval['t'+ str(i + 1)])
