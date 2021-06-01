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
        T_01 = self.TF_matrix(alpha1, a1, d1, q1).subs(self.dh_params)
        self.tf_matrices_list.append(T_01)
        T_12 = self.TF_matrix(alpha2, a2, d2, q2).subs(self.dh_params)
        self.tf_matrices_list.append(T_01 * T_12)
        T_23 = self.TF_matrix(alpha3, a3, d3, q3).subs(self.dh_params)
        self.tf_matrices_list.append(T_01 * T_12 * T_23)
        T_34 = self.TF_matrix(alpha4, a4, d4, q4).subs(self.dh_params)
        self.tf_matrices_list.append(T_01 * T_12 * T_23 * T_34)
        T_45 = self.TF_matrix(alpha5, a5, d5, q5).subs(self.dh_params)
        self.tf_matrices_list.append(T_01 * T_12 * T_23 * T_34 * T_45)
        T_56 = self.TF_matrix(alpha6, a6, d6, q6).subs(self.dh_params)
        T = T_01 * T_12 * T_23 * T_34 * T_45 * T_56
        self.tf_matrices_list.append(T)

    def show_transform_matrices(self):
        print('Transform Matrices are: {}'.format(self.tf_matrices_list))

    @staticmethod
    def TF_matrix(alpha, a, d, q):
        TF = Matrix([[cos(q), -cos(alpha) * sin(q), sin(q) * sin(alpha), a * cos(q)],
                     [sin(q), cos(alpha) * cos(q), -sin(alpha) * cos(q), a * sin(q)],
                     [0, sin(alpha), cos(alpha), d],
                     [0, 0, 0, 1]])
        return TF

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

        # TODO: calculate [x,y,z,roll,pitch,yaw]

        self.current_pos = np.array([x, y, z, roll, pitch, yaw])

        return self.current_pos

    def jacobian_func(self):
        """
        calculate jacobian function of the arm
        :param data:
        :type data:
        :return: jacobian_mat
        :rtype: 6x6 symbolic matrix
        """
        # homogenous transformation matrix from base_link to end_effector [type: syms]
        T_0G = self.tf_matrices_list[-1]

        # TODO: calculate the Jacobian matrix of the arm using the lecture notes ( symbolic matrix)
        # TODO: hint - you can use automatic differentiation ('diff(A,by_who)')

        self.jacobian_mat = Matrix([])# fill

        return self.jacobian_mat

    def inverse_kinematics(self, guess, target):
        """
        calculate inverse kinematics for a given gripper state configuration
        :param data: guess, target
        :type data:  list
        :return: jacobian_mat
        :rtype: 6x6 symbolic matrix
        """

        error = 10.0
        tolerance = 10.0

        Q = guess  # Initial Guess - Joint Angles
        target = np.matrix(target)  # X,Y,Z R,P,Y value for Target Position
        self.jacobian_func()

        error_grad = []
        theta_dict = {}
        lr = 0.1



        while error > tolerance:

            for i in range(len(Q)):
                theta_dict[self.q[i]] = Q[i]

            T_q = np.matrix(self.forward_kinematics(Q))

            delta_T = target - T_q # difference between current forward kinematics to the target

            # TODO: edit the update rule base on the lecture notes
            Q =

            prev_error = error

            error = LA.norm(delta_T)

            # if error > 10 * tolerance:
            #     lr = 0.3
            # elif error < 10 * tolerance:
            #     lr = 0.2

            error_grad.append((error - prev_error))

            print('error' + str(error))

        return Q

    def path_plan(self, guess, target_list):

        Q_list = []
        for i in range(len(target_list)):
            target = target_list['t' + str( i + 1)]
            Q = self.inverse_kinematics(guess, target)
            predicted_coordinates = self.forward_kinematics(Q)
            print('Target: {} ,  Predicted: {}'.format(target, predicted_coordinates))
            Q_list.append(Q)
            guess = Q
        Q_matrix = np.matrix(Q_list)
        return Q_matrix


def move_to_angle_conf(Q):


    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../Lab1/"))
    from lab_01_single_action import example_move_to_home_position, example_cartesian_action_movement, example_angular_action_movement
    import utilities

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

def move_to_gripper_conf(C):

    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../Lab1/"))
    from lab_01_single_action import example_move_to_home_position, example_cartesian_action_movement, example_angular_action_movement
    import utilities

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
                      "Press A to move the arm to desired gripper conf: " + str(Q) + '\n'
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
                success &= example_cartesian_action_movement(base,C )
                if success:
                    print('Successfully moved to arm to desired angular action')
                    display = True
                else:
                    print('Huston, we have a problem, please call the instructor')

if __name__ == '__main__':

    # For simplicity, we used symbolic function for differentiation
    # A tutorial can be found here: https://docs.sympy.org/latest/tutorial/index.html

    #######################
    ###### Part 1 #########
    #######################

    arm = robotic_arm()     # Initialize a generic robotic arm
    arm.set_joints(6)       # Define the amount of joints

    # DH params
    alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
    a1, a2, a3, a4, a5, a6 = symbols('a1:7')
    d1, d2, d3, d4, d5, d6 = symbols('d1:7')
    q1, q2, q3, q4, q5, q6 = symbols('q1:7')

    # TODO: fill the following dict as described in the lab notes

    dh_subs_dict = {alpha1:,  a1: , d1: , q1: ,
                    alpha2: , a2: , d2: , q2: ,
                    alpha3: , a3: , d3: , q3: ,
                    alpha4: , a4: , d4: , q4: ,
                    alpha5: , a5: , d5: , q5: ,
                    alpha6: , a6: , d6: , q6: } # [mm - radians]


    arm.set_dh_param_dict(dh_subs_dict) # set dh paramters

    # TODO: enter the desired joint state target configrations
    angle_conf_target_dict = {'t1':  [ , , , , , ] ,
                              't2':  [ , , , , , ] ,
                              't3':  [ , , , , , ] ,
                              't4':  [ , , , , , ] ,
                              't5':  [ , , , , , ]} # [deg]
    angle_conf_eval = {}

    # TODO: fill the blank line in the forward_kinematics function
    for i in range(len(angle_conf_target_dict)):
        angle_conf_eval.update({'t'+ str(i + 1): arm.forward_kinematics(angle_conf_target_dict['t' + str(i + 1)])})

    # Import arm modules and move to each configuration
    for i in range(len(angle_conf_eval)):
        move_to_angle_conf(angle_conf_eval['t'+ str(i + 1)])


    #######################
    ###### Part 2 #########
    #######################

    # TODO: enter the desired joint state target configurations
    #                                  x   y   z    roll  pitch  yaw
    gripper_conf_target_dict = {'t1': [ , , , , , ],
                                't2': [ , , , , , ],
                                't3': [ , , , , , ],
                                't4': [ , , , , , ],
                                't5': [ , , , , , ]}

    # TODO: enter inital guess for the arm configuration
    init_guess = [ , , , , , ]

    gripper_conf_eval = {}
    guess = init_guess

    # TODO: fill the missing lines in the inverse_kinematics function
    for i in range(len(gripper_conf_target_dict)):
        target = gripper_conf_target_dict['t' + str(i + 1)]
        Q = arm.inverse_kinematics(guess, target)
        predicted_coordinates = arm.forward_kinematics(Q)
        print('Target: {} ,  Predicted: {}'.format(target, Q))
        gripper_conf_eval.update({'t' + str(i + 1): Q})
        guess = Q

    # Import arm modules and move to each configration
    for i in range(len(angle_conf_eval)):
        move_to_gripper_conf(gripper_conf_eval['t'+ str(i + 1)])