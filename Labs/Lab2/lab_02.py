import numpy as np
from sympy import *
from numpy import linalg as LA
from numpy.linalg import inv, pinv
from scipy.linalg import logm
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
        self.tf_matrices_list.append(T_01 * T_12 * T_23 * T_34 * T_45*T_56)
        Tes = Matrix([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        T = T_01 * T_12 * T_23 * T_34 * T_45 * T_56 * Tes
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
        # Euler = R.as_euler('xyz', degrees=False)
        roll = [np.array(Euler[0]).astype(np.float64)]
        pitch = [np.array(Euler[1]).astype(np.float64)]
        yaw = [np.array(Euler[2]).astype(np.float64)]
        self.current_pos.append(np.array([x, y, z, roll, pitch, yaw]))

        return self.current_pos

    def forward_hom_mat(self, theta_list):
        theta_dict = {}
        T_0G = self.tf_matrices_list[-1]

        for i in range(len(theta_list)):
            theta_dict[self.q[i]] = theta_list[i]

        return T_0G.evalf(subs=theta_dict, chop=True, maxn=4)

    def jacobian_func(self):
        T_0G = self.tf_matrices_list[-1]

        self.jacobian_mat = [diff(T_0G[:3, -1], self.q[i]).reshape(1, 3) for i in range(len(self.q))]
        # self.jacobian_mat = np.vstack((self.jacobian_mat, ))
        self.jacobian_mat = Matrix(self.jacobian_mat).T
        # to_jac = [list(A[:3, 2]) for A in self.tf_matrices_list]
        temp = Matrix([0, 0, 1])
        for index in range(len(self.tf_matrices_list)-2):
            temp = temp.col_insert(index+1, self.tf_matrices_list[index][:3, 2]) #index+1

        self.jacobian_mat = Matrix(BlockMatrix([[self.jacobian_mat], [temp]]))

    def inverse_kinematics(self, guess, target):
        error = 0.15
        tolerance = 0.1

        # Initial Guess - Joint Angles
        Q = guess
        # X,Y expression
        # X,Y,Z R,P,Y value for Target Position
        target = np.matrix(target)
        print(target.shape)
        # Init Jacobian
        self.jacobian_func()
        # T_0G = self.tf_matrices_list[-1]

        error_grad = []

        theta_dict = {}
        theta_dict_deg = {}

        lr = 0.01

        while error > tolerance:

            # lr = error * 0.1
            # Q_deg = [np.rad2deg(x) for x in Q]
            for i in range(len(Q)):
                theta_dict[self.q[i]] = Q[i]
                # theta_dict_deg[self.q[i]] = Q_deg[i]

            T_q = np.matrix(self.forward_kinematics(Q)[-1])
            # print(Q)
            delta_T = target.T - T_q
            inv_sol = inv(np.matrix(self.jacobian_mat.evalf(subs=theta_dict, chop=True, maxn=4)).astype(np.float64)).T
            Q = Q + lr * (inv_sol * delta_T).reshape(-1) # .T
            Q = Q.tolist()[0] # np.array(Q)[0]

            prev_error = error

            error = LA.norm(delta_T)

            # if error > 10 * tolerance:
            #     lr = 0.5
            # elif error < 10 * tolerance:
            #     lr = 0.2

            error_grad.append((error - prev_error))

            print(error) # error

        return Q

    def inverse_kinematics_book(self, guess, target):
        error = 0.15
        tolerance = 0.1

        # Initial Guess - Joint Angles
        Q = guess
        # X,Y expression
        # X,Y,Z R,P,Y value for Target Position
        target2 = np.matrix(target)
        xyz = target2[0,:3]
        target2 = Rotation.from_euler('xyz',target2[0,3:],degrees=False)
        target2 = target2.as_matrix()[0, :,:]
        target2 = np.vstack((np.concatenate((target2, xyz.T), axis=1), np.array([0,0,0,1])))
        # Init Jacobian
        self.jacobian_func()
        # T_0G = self.tf_matrices_list[-1]

        error_grad = []

        theta_dict = {}
        theta_dict_deg = {}

        lr = 0.1

        while error > tolerance:

            # lr = error * 0.1
            # Q_deg = [np.rad2deg(x) for x in Q]
            for i in range(len(Q)):
                theta_dict[self.q[i]] = Q[i]
                # theta_dict_deg[self.q[i]] = Q_deg[i]

            T_q = np.matrix(self.forward_hom_mat(Q)).astype(np.float64)
            V = inv(T_q) * T_q
            Vb = logm(V)
            # print(Q)
            delta_T = target2.T - T_q
            inv_sol = pinv(np.matrix(self.jacobian_mat.evalf(subs=theta_dict, chop=True, maxn=4)).astype(np.float64)).T
            J = np.matrix(self.jacobian_mat.evalf(subs=theta_dict, chop=True, maxn=4)).astype(np.float64)
            omega = Rotation.from_matrix(Vb[:3, :3])
            omega = omega.as_euler('xyz')
            v = Vb[:3, -1]

            Q = Q + lr * (J * delta_T).reshape(-1) # .T

            # Q = Q + lr * ((inv_sol/np.linalg.norm(inv_sol)) * np.concatenate((v, omega)).reshape(6, 1)).reshape(-1) # .T
            Q = Q.tolist()[0] # np.array(Q)[0]

            prev_error = error

            error = LA.norm(delta_T)

            # if error > 10 * tolerance:
            #     lr = 0.5
            # elif error < 10 * tolerance:
            #     lr = 0.2

            error_grad.append((error - prev_error))

            print(error) # error

        return Q

    def path_plan(self, guess, target_list):
        Q_list = []
        for i in range(len(target_list)):
            target = target_list['t' + str( i + 1)]
            Q = self.inverse_kinematics(guess, target)
            predicted_coordinates = self.forward_kinematics(Q)
            print('Target: {} ,  Predicted: {}'.format(target, predicted_coordinates[-1]))
            Q_list.append(Q)
            guess = Q
        Q_matrix = np.matrix(Q_list)
        return Q_matrix


def move_to_angle_conf(Q, base, base_cyclic):


    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../Lab1/"))
    from lab_01_single_action import example_move_to_home_position, example_cartesian_action_movement, example_angular_action_movement
    import utilities



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
            success &= example_angular_action_movement(base, base_cyclic, Q)
            if success:
                print('Successfully moved to arm to desired angular action')
                display = True
            else:
                print('Huston, we have a problem, please call the instructor')
        break

def move_to_gripper_conf(C, base, base_cyclic):

    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../Lab1/"))
    from lab_01_single_action import example_move_to_home_position, example_cartesian_action_movement, example_angular_action_movement
    import utilities

    input("Remove any objects near the arm and press Enter")
    # Example core
    success = True
    flag = True
    display = True

    while flag and success:

        if display:
            key = input("Press H to move the arm  to home position\n"
                  "Press A to move the arm to desired gripper conf: " + str(C) + '\n'
                  "To Quit press Q")
            display = False

        if str(key) == 'h' or str(key) == 'H':
            success &= example_move_to_home_position(base)
            if success:
                print('Successfully moved to home position')
                display = True
            else:
                print('Huston, we have a problem, please call the instructor')

        if str(key) == 'A' or str(key) == 'a':
            success &= example_cartesian_action_movement(base, base_cyclic, C)
            if success:
                print('Successfully moved to arm to desired angular action')
                display = True

            else:
                print('Huston, we have a problem, please call the instructor')
        break

if __name__ == '__main__':
    ## Init Robot
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


        #######################
        ###### Part 1 #########
        #######################

        arm = robotic_arm()
        arm.set_joints(6)
        #               alpha a d r
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
        a1, a2, a3, a4, a5, a6 = symbols('a1:7')
        d1, d2, d3, d4, d5, d6 = symbols('d1:7')
        q1, q2, q3, q4, q5, q6 = symbols('q1:7')

        dh_subs_dict = {alpha1: pi / 2, a1: 0, d1: 0.1283 + 0.1150, q1: q1,
                         alpha2: pi, a2: 0.280, d2: 0.030, q2: q2 + pi / 2,
                         alpha3: pi / 2, a3: 0, d3: 0.020, q3: q3 + pi / 2,
                         alpha4: pi / 2, a4: 0, d4: 0.1400 + 0.1050, q4: q4 + pi / 2,
                         alpha5: pi / 2, a5: 0, d5: 0.0285 + 0.0285, q5: q5 + pi,
                         alpha6: 0, a6: 0, d6: 0.1050 + 0.130, q6: q6 + pi / 2}

        arm.set_dh_param_dict(dh_subs_dict)

        angle_conf_target_dict = {'t1': [0,0,0,0,0,0], # deg
                                  't2': [pi/2,0,0,pi/4,pi/4,pi/4],
                                  't3': [0,np.deg2rad(344),np.deg2rad(75),0,np.deg2rad(300),0],
                                  't4': [7,21,150,285,340,270],
                                  't5': [0,0,0,0,0,0]}


        angle_conf_eval = {}

        for i in range(len(angle_conf_target_dict)):
            angle_conf_eval.update({'t'+ str(i + 1): arm.forward_kinematics(angle_conf_target_dict['t' + str(i + 1)])[-1]})
        #
        # # Import arm modules and move to each configration
        # for i in range(len(angle_conf_eval)):
        #     move_to_gripper_conf(angle_conf_eval['t'+ str(i + 1)], base, base_cyclic)

        #######################
        ###### Part 2 #########
        #######################

        #                                  x   y   z    roll  pitch  yaw
        gripper_conf_target_dict = {'t2': [0.057, -0.01, 1.0033, 0., 0.0, 0.],
                                    't1': [-0.03017, 0.2087, 0.8917, -0.6500, -1.0529, -1.9830],
                                    't3': [0.5233, 0.2249, 0.2814, -1.3889, 0.9066, 0.9269],
                                    't4': [-0.3851, -0.1620, -0.3096, -1.5797, -1.8567, 0.8468],
                                    't5': [0.057, -0.01, 1.0033, 0., 0.0, 0.]}

        init_guess = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
        gripper_conf_eval = {}
        guess = init_guess
        for i in range(len(gripper_conf_target_dict)):
            target = gripper_conf_target_dict['t' + str(i + 1)]
            Q = arm.inverse_kinematicsV2(guess, target)
            predicted_coordinates = arm.forward_kinematics(Q)[-1]
            print('Target: {} ,  Predicted: {}, Angles: {}'.format(target, predicted_coordinates, Q))
            gripper_conf_eval.update({'t' + str(i + 1): Q})
            guess = Q

        # Import arm modules and move to each configration
        for i in range(len(angle_conf_eval)):
            move_to_gripper_conf(gripper_conf_eval['t'+ str(i + 1)], base, base_cyclic)