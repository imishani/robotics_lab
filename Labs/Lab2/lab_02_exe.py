from sympy import *
import os
import numpy as np
if os.name == 'nt':
    import msvcrt
else:
    import tty, termios
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2
import sys
# from lab02_student import *
from lab02_solution import *

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../Lab1/"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../common/robot"))
from robot_actions import *
import utilities
np.set_printoptions(precision=2, suppress=True, threshold=5)

class robotic_arm_lab2():

    def __init__(self):
        self.joints = 0
        self.alpha = None
        self.a = None
        self.q = None
        self.d = None
        self.dh_params = {}
        self.T = None

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
        self.T = transform_matrices()

    def forward_kinematics_sympy(self, theta_list):
        """
        calculate forward kinematic for a desired theta_list

        :param data: theta_list
        :type data: list
        :return: forward kinematics list SE-3  of the form [x y z roll pitch yaw] w.r.t base_link
        :rtype: list
        """

        theta_dict = {}
        '''homogenous transformation matrix from base_link to end_effector [type: syms]'''

        for i in range(len(theta_list)):
            theta_dict[self.q[i]] = theta_list[i]

        '''homogenous transformation matrix from base_link to end_effector [type: numeric matrix] '''
        T_0G_eval = self.T.evalf(subs=theta_dict, chop=True, maxn=4)

        '''calculate [x,y,z,roll,pitch,yaw] '''

        self.current_pos = xyz_rpy(T_0G_eval)

        return self.current_pos


    def forward_kinematics(self, theta_list):
        """
        calculate forward kinematic for a desired theta_list

        :param data: theta_list
        :type data: list
        :return: forward kinematics list SE-3  of the form [x y z roll pitch yaw] w.r.t base_link
        :rtype: list
        """

        '''homogenous transformation matrix from base_link to end_effector [type: numeric matrix] '''
        T_0G_eval = FK(theta_list)

        '''calculate [x,y,z,roll,pitch,yaw] '''

        self.current_pos = xyz_rpy(T_0G_eval)


        return self.current_pos


def move_to_angle_conf(angle_conf_eval):

    args = utilities.parseConnectionArguments()
    with utilities.DeviceConnection.createTcpConnection(args) as router:
        # Create required services
        base = BaseClient(router)
        base_cyclic = BaseCyclicClient(router)
        input("Remove any objects near the arm and press Enter")
        input("Moving the robot to home position")
        example_move_to_home_position(base)

        quit_flag = False
        for i in range(len(angle_conf_eval)):

            C = angle_conf_eval['t'+ str(i + 1)]

            success = True
            flag = True
            display = True

            while flag and success and not quit_flag:

                if display:
                    key = input("Press H to move the arm  to home position\n"
                                "Press A to move the arm to desired cartesian position: \n"
                                + str(np.round(C.squeeze(), 3))+ '\n'
                                + "To Quit press Q\n")
                    display = False

                if str(key) == 'h' or str(key) == 'H':
                    success &= example_move_to_home_position(base)
                    if success:
                        print('Successfully moved to home position')
                        display = True
                    else:
                        print('Huston, we have a problem, please call the instructor')

                if str(key) == 'A' or str(key) == 'a':
                    success &= example_cartesian_action_movement(base, base_cyclic, C=C)
                    if success:
                        print('Successfully moved to arm to desired angular action')
                        flag = False
                    else:
                        print('Huston, we have a problem, please call the instructor')
                if str(key) == 'q' or str(key) == 'Q':
                    quit_flag = True
                    break


if __name__ == '__main__':
    # For simplicity, we used symbolic function for differentiation
    # A tutorial can be found here: https://docs.sympy.org/latest/tutorial/index.html

    arm = robotic_arm_lab2()     # Initialize a generic robotic arm
    arm.set_joints(6)       # Define the amount of joints

    # TODO: fill the following dict as described in the lab notes

    arm.set_dh_param_dict(set_dh_table())

    # TODO: enter the desired joint state target configurations
    angle_conf_target_dict = angles_to_follow()
    angle_conf_eval = {}

    # TODO: fill the blank line in the forward_kinematics function
    for i in range(len(angle_conf_target_dict)):
        angle_conf_eval.update({'t' + str(i + 1): arm.forward_kinematics(angle_conf_target_dict['t' + str(i + 1)])})

    ''' Import arm modules and move to each configuration '''
    print('Evaluation of the target angle configurations:\n')
    print('         X    Y     Z    roll    pitch   yaw')
    for target in angle_conf_eval:
        print(str(target) + '    ' + str(angle_conf_eval[target].reshape(-1)))
    print()

    move_to_angle_conf(angle_conf_eval)

