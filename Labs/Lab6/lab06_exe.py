import time

from sympy import *
# import sympy
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
from lab06_solution import *

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../Lab1/"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../common/pyFT300"))
from pyFT300stream import *
from FT300_mishani import *
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

def prop_k_estimation(base_cyclic, ):

    theta_dict = {}
    cur_joint = np.zeros(len(base_cyclic.RefreshFeedback().actuators))
    cur_torque = np.zeros(len(base_cyclic.RefreshFeedback().actuators))
    cur_current = np.zeros(len(base_cyclic.RefreshFeedback().actuators))
    Kt = np.zeros(len(base_cyclic.RefreshFeedback().actuators))
    Kt_sum = np.zeros(len(base_cyclic.RefreshFeedback().actuators))

    ################# Part  1
    """
    Current-based torque estimation
    tau = K_t*I 
    torque is proportinal to the current at each motor
    For each of the motors, find Kt such as K_t = tau/I
    
    """
    #################
    c=0
    while(True):
        for i in range(len(base_cyclic.RefreshFeedback().actuators)):
            cur_joint[i] = base_cyclic.RefreshFeedback().actuators[i].position
            theta_dict.update({'q' + str(i + 1): cur_joint[i]})
            cur_torque[i] = base_cyclic.RefreshFeedback().actuators[i].torque
            cur_current[i] = base_cyclic.RefreshFeedback().actuators[i].current_motor
            Kt[i] = cur_torque[i]/cur_current[i]
            print("Kt " + str(i+1) + " : " + str(Kt[i]))
            c+=1
        Kt_sum += Kt
        print("Kt: " + str(Kt_sum / c))

def static_load(base_cyclic, ft):

    #################
    ### External-force Torque estimation
    ### tau = -J.T * F
    #################
    theta_dict = {}
    cur_joint = np.zeros(len(base_cyclic.RefreshFeedback().actuators))
    cur_torque = np.zeros(len(base_cyclic.RefreshFeedback().actuators))
    # ft = FT300Sensor()
    # ft.init_connection()
    # while True:
    #     F = ft.get_reading()
    #     print(F)
    ft = ft_sens()
    ft_bias = None
    while True:
        for i in range(len(base_cyclic.RefreshFeedback().actuators)):
            cur_joint[i] = base_cyclic.RefreshFeedback().actuators[i].position
            cur_torque[i] = base_cyclic.RefreshFeedback().actuators[i].torque
            theta_dict.update({'q' + str(i + 1): np.deg2rad(cur_joint[i])})
        J = np.matrix(arm.jacobian_mat.evalf(subs=theta_dict, chop=True, maxn=10)).astype(np.float64)
        F = ft.get_forces()
        # F = ft.get_reading()  # Fx,Fy,Fz, Mx,My,Mz
        tau = -J.T.dot(F)  # estimated
        error = tau - cur_torque
        if ft_bias is None:
            ft_bias = cur_torque.copy()
        print("F: " + str(F))
        print("cur_torque: " + str((cur_torque - ft_bias).tolist()))
        print("cur_tau: " + str(tau.tolist()))

def dynamic_test(base, base_cyclic):
    success = True
    try:
        success &= move_to_zero_position(base)
    except:
        print('Could not reach zero position')
    path = 'trajectory.csv'
    M = np.genfromtxt(path, delimiter=',')
    N = M.shape[0]
    couple = np.zeros((N, 6))
    q = M[:, 1:7]
    qp = M[:, 7:13]
    qpp = M[:, 13:19]
    t = M[:, 0]

    # for i in range(N):
    #     q = M[i, 1:7]
    #     qp = M[i, 7:13]
    #     qpp = M[i, 13:19]
    #     # couple[i, 0:6] = dyn.precomputed_torque(q, qp, qpp).T

    recorded_torques = np.array([[base_cyclic.RefreshFeedback().actuators[i].torque
                                  for i in range(len(base_cyclic.RefreshFeedback().actuators))]])
    itamar = Base_pb2.PreComputedJointTrajectory()
    for j in range(q.shape[0]):
        elem = itamar.trajectory_elements.add()
        elem.time_from_start = t[j]
        elem.joint_angles.extend(q[j, :].T)
        elem.joint_speeds.extend(qp[j, :].T)
        elem.joint_accelerations.extend(qpp[j, :].T)
    time_ = time.time()
    base.PlayPreComputedJointTrajectory(itamar)

    while time.time() - time_ < (t[-1] + 2):
        recorded_torques = np.vstack((recorded_torques, np.array([base_cyclic.RefreshFeedback().actuators[i].torque
                                                                  for i in range(
                len(base_cyclic.RefreshFeedback().actuators))])))



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

            # dh_subs_dict = {alpha1: pi / 2, a1: 0, d1: 128.3 + 115.0, q1: q1,
            #                 alpha2: pi, a2: 280, d2: 30, q2: q2 + pi / 2,
            #                 alpha3: pi / 2, a3: 0, d3: 20, q3: q3 + pi / 2,
            #                 alpha4: pi / 2, a4: 0, d4: 140.0 + 105.0, q4: q4 + pi / 2,
            #                 alpha5: pi / 2, a5: 0, d5: 28.5 + 28.5, q5: q5 + pi,
            #                 alpha6: 0, a6: 0, d6: 105.0 + 130.0, q6: q6 + pi / 2}
            dh_subs_dict = {alpha1: pi / 2, a1: 0, d1: 0.1283 + 0.115, q1: q1,
                            alpha2: pi, a2: 0.28, d2: 0.030, q2: q2 + pi / 2,
                            alpha3: pi / 2, a3: 0, d3: 0.020, q3: q3 + pi / 2,
                            alpha4: pi / 2, a4: 0, d4: 0.140 + 0.105, q4: q4 + pi / 2,
                            alpha5: pi / 2, a5: 0, d5: 0.0285 + 0.0285, q5: q5 + pi,
                            alpha6: 0, a6: 0, d6: 0.1050 + 0.130, q6: q6 + pi / 2}

            arm.set_dh_param_dict(dh_subs_dict)
            # Init Jacobian
            arm.jacobian_func()

            # init ft
            ft=False
            # ft = FT300Sensor()
            # ft.init_connection()
            # while True:
            #     cur_ft = ft.get_reading()
            #     print(cur_ft)

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

                    success &= static_load(base_cyclic, ft)
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
