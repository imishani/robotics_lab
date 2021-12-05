#! /usr/bin/env python3

###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2018 Kinova inc. All rights reserved.
#
# This software may be modified and distributed
# under the terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.
#
from sympy import *
from numpy import linalg as LA
from numpy.linalg import inv
import time
import logging
from scipy.spatial.transform import Rotation
import math
import os
import numpy as np
import sys
import os
import time
import threading

import sys, select, os

if os.name == 'nt':
    import msvcrt
else:
    import tty, termios

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2
import signal
import sys
import time
import threading

# Maximum allowed waiting time during actions (in seconds)
TIMEOUT_DURATION = 20
e = """
Communications Failed
"""


def getKey():
    if os.name == 'nt':
        return msvcrt.getch()
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    return True


def check_for_end_or_abort(e):
    """Return a closure checking for END or ABORT notifications

    Arguments:
    e -- event to signal when the action is completed
        (will be set when an END or ABORT occurs)
    """

    def check(notification, e=e):
        print("EVENT : " + \
              Base_pb2.ActionEvent.Name(notification.action_event))
        if notification.action_event == Base_pb2.ACTION_END \
                or notification.action_event == Base_pb2.ACTION_ABORT:
            e.set()

    return check


def example_move_to_home_position(base):
    # Make sure the arm is in Single Level Servoing mode
    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)

    # Move arm to ready position
    print("Moving the arm to a safe position")
    action_type = Base_pb2.RequestedActionType()
    action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
    action_list = base.ReadAllActions(action_type)
    action_handle = None
    for action in action_list.action_list:
        if action.name == "Home":
            action_handle = action.handle

    if action_handle == None:
        print("Can't reach safe position. Exiting")
        return False

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    base.ExecuteActionFromReference(action_handle)
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("Safe position reached")
    else:
        print("Timeout on action notification wait")
    return finished


class robotic_arm():

    def __init__(self):
        self.joints = 0
        self.alpha = None
        self.a = None
        self.q = None
        self.d = None
        self.dh_params = {}
        self.tf_matrices_list = []

        self.current_pos = []

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
        theta_dict = {}
        T_0G = self.tf_matrices_list[-1]

        for i in range(len(theta_list)):
            theta_dict[self.q[i]] = theta_list[i]

        temp = T_0G.evalf(subs=theta_dict, chop=True, maxn=4)
        x = [np.array(temp[0, -1]).astype(np.float64)]
        y = [np.array(temp[1, -1]).astype(np.float64)]
        z = [np.array(temp[2, -1]).astype(np.float64)]
        R = Rotation.from_matrix(temp[:3, :3])
        Euler = R.as_rotvec()
        roll = [np.array(Euler[0]).astype(np.float64)]
        pitch = [np.array(Euler[1]).astype(np.float64)]
        yaw = [np.array(Euler[2]).astype(np.float64)]
        self.current_pos.append(np.array([x, y, z, roll, pitch, yaw]))

        return self.current_pos

    def jacobian_func(self):
        T_0G = self.tf_matrices_list[-1]

        self.jacobian_mat = [diff(T_0G[:3, -1], self.q[i]).reshape(1, 3) for i in range(len(self.q))]
        # self.jacobian_mat = np.vstack((self.jacobian_mat, ))
        self.jacobian_mat = Matrix(self.jacobian_mat).T
        # to_jac = [list(A[:3, 2]) for A in self.tf_matrices_list]
        temp = Matrix([0, 0, 1])
        for index in range(len(self.tf_matrices_list) - 1):
            temp = temp.col_insert(0, self.tf_matrices_list[index][:3, 2])

        self.jacobian_mat = Matrix(BlockMatrix([[self.jacobian_mat], [temp]]))

    def forward_hom_mat(self, theta_list):
        theta_dict = {}
        T_0G = self.tf_matrices_list[-1]

        for i in range(len(theta_list)):
            theta_dict[self.q[i]] = theta_list[i]

        return T_0G.evalf(subs=theta_dict, chop=True, maxn=4)

    def inverse_kinematics(self, guess, target):
        error = 10.0
        tolerance = 10.0

        # Initial Guess - Joint Angles
        Q = guess
        # X,Y expression
        # X,Y,Z R,P,Y value for Target Position
        target = np.matrix(target)
        print(target.shape)
        # Jacobian
        self.jacobian_func()
        # T_0G = self.tf_matrices_list[-1]

        error_grad = []

        theta_dict = {}

        lr = 0.1

        while error > tolerance:

            for i in range(len(Q)):
                theta_dict[self.q[i]] = Q[i]

            T_q = np.matrix(self.forward_kinematics(Q)[-1])

            delta_T = target - T_q

            Q = Q + lr * (inv(np.matrix(self.jacobian_mat.evalf(subs=theta_dict, chop=True, maxn=10)).astype(
                np.float64)) * delta_T).reshape(-1)
            Q = Q.tolist()[0]

            prev_error = error

            error = LA.norm(delta_T)

            # if error > 10 * tolerance:
            #     lr = 0.3
            # elif error < 10 * tolerance:
            #     lr = 0.2

            error_grad.append((error - prev_error))

            print(error)

        return Q

    def path_plan(self, guess, target_list):
        Q_list = []
        for i in range(len(target_list)):
            target = target_list['t' + str(i + 1)]
            Q = self.inverse_kinematics(guess, target)
            predicted_coordinates = self.forward_kinematics(Q)
            print('Target: {} ,  Predicted: {}'.format(target, predicted_coordinates[-1]))
            Q_list.append(Q)
            guess = Q
        Q_matrix = np.matrix(Q_list)
        return Q_matrix


def static_load(base, base_cyclic):


    theta_dict = {}
    cur_joint = np.zeros(len(base_cyclic.RefreshFeedback().actuators))
    cur_torque = np.zeros(len(base_cyclic.RefreshFeedback().actuators))
    cur_current = np.zeros(len(base_cyclic.RefreshFeedback().actuators))
    Kt = np.zeros(len(base_cyclic.RefreshFeedback().actuators))
    Kt_sum = np.zeros(len(base_cyclic.RefreshFeedback().actuators))

    ################# Part 1
    """
    Current-based torque estimation
    tau = K_t*I - F
    torque is proportinal to the current at each motor
    For each of the motors, find Kt such as K_t = tau/I
    """
    #################
    c = 0
    while(False):
        c += 1
        print("Kt: " + str(Kt_sum/c))

    #################
    ### External-force Torque estimation
    ### tau = -J.T * F
    #################

    J = np.matrix(arm.jacobian_mat.evalf(subs=theta_dict, chop=True, maxn=10)).astype(np.float64)
    F = np.array([0.0, 0.0])    # Todo: get F by external force sensor
    tau = -J.T.dot(F)  # estimated
    error = tau - cur_torque

    #################
    ### Dynamic-Modeling
    """
    tau = M(q)q_ddot + C(q,q_dot)q_dot + g(q)
    """
    ################



if __name__ == "__main__":

    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
    try:
        # base_cyclic = main()
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
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

            arm = robotic_arm()
            arm.set_joints(6)
            #               alpha a d r
            alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
            a1, a2, a3, a4, a5, a6 = symbols('a1:7')
            d1, d2, d3, d4, d5, d6 = symbols('d1:7')
            q1, q2, q3, q4, q5, q6 = symbols('q1:7')

            dh_subs_dict = {alpha1: pi / 2, a1: 0, d1: 128.3 + 115.0, q1: q1,
                            alpha2: pi, a2: 280, d2: 30, q2: q2 + pi / 2,
                            alpha3: pi / 2, a3: 0, d3: 20, q3: q3 + pi / 2,
                            alpha4: pi / 2, a4: 0, d4: 140.0 + 105.0, q4: q4 + pi / 2,
                            alpha5: pi / 2, a5: 0, d5: 28.5 + 28.5, q5: q5 + pi,
                            alpha6: 0, a6: 0, d6: 105.0 + 130.0, q6: q6 + pi / 2}

            arm.set_dh_param_dict(dh_subs_dict)
            # Init Jacobian
            arm.jacobian_func()

            while flag and success:

                if display:
                    key = input("Press H to move the arm  to home position\n"
                                "Press C to follow a start static loading experiment\n"
                                "Press A do somthing\n"
                                "To Quit press Q")
                    display = False

                if str(key) == 'h' or str(key) == 'H':
                    success &= example_move_to_home_position(base)
                    if success:
                        print('Successfully moved to home position')
                        display = True
                    else:
                        print('Huston, we have a problem, please call the instructor')

                if str(key) == 'c' or str(key) == 'C':

                    success &= static_load(base, base_cyclic)
                    if success:
                        print('Successfully moved')
                        display = True
                    else:
                        print('Huston, we have a problem, please call the instructor')

                if str(key) == 'A' or str(key) == 'a':
                    success &= TOEDIT(base, waypoints)
                    if success:
                        print('Successfully moved to arm to desired angular action')
                        display = True
                    else:
                        print('Huston, we have a problem, please call the instructor')

        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    except KeyboardInterrupt:
        pass
