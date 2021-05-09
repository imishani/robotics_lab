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
        for index in range(len(self.tf_matrices_list)-1):
            temp = temp.col_insert(0, self.tf_matrices_list[index][:3, 2])

        self.jacobian_mat = Matrix(BlockMatrix([[self.jacobian_mat], [temp]]))

    def inverse_kinematics(self, guess, target):
        error = 10.0
        tolerance = 0.5

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

        lr = 0.2
        while error > tolerance:
            for i in range(len(Q)):
                theta_dict[self.q[i]] = Q[i]

            T_q = np.matrix(self.forward_kinematics(Q)[-1])

            delta_T = target - T_q
            # print((Q + lr * (inv(np.matrix(self.jacobian_mat.evalf(subs=theta_dict, chop=True, maxn=10)).astype(
            #     np.float64)) * delta_T).reshape(-1)).shape)
            Q = Q + lr * (inv(np.matrix(self.jacobian_mat.evalf(subs=theta_dict, chop=True, maxn=10)).astype(
                np.float64)) * delta_T).reshape(-1)
            Q = Q.tolist()[0]

            prev_error = error

            error = LA.norm(delta_T)

            if error > 10 * tolerance:
                lr = 0.3
            elif error < 10 * tolerance:
                lr = 0.2
            error_grad.append((error - prev_error))

            print(error)
        return Q

    def path_plan(self, guess, target_list, time, acceleration):
        Q_list = []
        for target in target_list:
            Q = self.inverse_kinematics(guess, target)
            predicted_coordinates = self.forward_kinematics(Q)
            print('Target: {} ,  Predicted: {}'.format(target, predicted_coordinates[-1]))
            Q_list.append(Q)
            guess = Q
        # print(np.matrix(Q_list), np.matrix(Q_list).shape)
        Q_matrix = np.matrix(Q_list)
        # theta_all, omega_all = lpsb.trajectory_planner(Q_matrix, time, acceleration, 0.01)
    # return Q_list


def init():
    line.set_data([], [])
    return line,


def animate(i):
    global transform_matrices, q, Q_list

    x = [np.array(transform_matrices[k].evalf(
        subs={q[0]: Q_list[i][0], q[1]: Q_list[i][1], q[2]: Q_list[i][2], q[3]: Q_list[i][3]}, chop=True, maxn=4)[
                      0, -1]).astype(np.float64) for k in range(len(transform_matrices))]
    y = [np.array(transform_matrices[k].evalf(
        subs={q[0]: Q_list[i][0], q[1]: Q_list[i][1], q[2]: Q_list[i][2], q[3]: Q_list[i][3]}, chop=True, maxn=4)[
                      1, -1]).astype(np.float64) for k in range(len(transform_matrices))]
    z = [np.array(transform_matrices[k].evalf(
        subs={q[0]: Q_list[i][0], q[1]: Q_list[i][1], q[2]: Q_list[i][2], q[3]: Q_list[i][3]}, chop=True, maxn=4)[
                      2, -1]).astype(np.float64) for k in range(len(transform_matrices))]

    line.set_data(np.array(x), np.array(y))
    line.set_3d_properties(np.array(z))
    return line,


if __name__ == '__main__':

    arm = robotic_arm()
    arm.set_joints(6)
    #                       alpha a d r
    alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
    a1, a2, a3, a4, a5, a6 = symbols('a1:7')
    d1, d2, d3, d4, d5, d6 = symbols('d1:7')
    q1, q2, q3, q4, q5, q6 = symbols('q1:7')

    subs_dict = {alpha1: pi / 2, a1: 0, d1: 128.3 + 115.0, q1: q1,
                 alpha2: pi, a2: 280, d2: 30, q2: q2 + pi / 2,
                 alpha3: pi / 2, a3: 0, d3: 20, q3: q3 + pi / 2,
                 alpha4: pi / 2, a4: 0, d4: 140.0 + 105.0, q4: q4 + pi / 2,
                 alpha5: pi / 2, a5: 0, d5: 28.5 + 28.5, q5: q5 + pi,
                 alpha6: 0, a6: 0, d6: 105.0 + 130.0, q6: q6 + pi / 2}

    arm.set_dh_param_dict(subs_dict)

    # trial = arm.forward_kinematics([57, -10, 1003.3, 0., 0.0, 0.])

    transform_matrices = arm.tf_matrices_list
    q = arm.q
    target_list = [[[57], [-10], [1003.3], [0.], [0.0], [1.]]] #, [[1.4], [1.0], [0.4]], [[1.2], [1.0], [0.2]], [[1.0], [1.0], [0.0]]]
    time = np.array([10, 10, 20])

acceleration = np.array([40, 35, 60, 50])
Q_list = arm.path_plan([1.5, 1.5, 0., 0.,2.,0.], target_list, time, acceleration)
# print(Q_list)


fig = plt.figure()
ax = p3.Axes3D(fig)
for i in range(len(target_list)):
    ax.scatter(target_list[i][0], target_list[i][1], target_list[i][2], c='r', marker='*')
ax.set_xlim3d([0.0, 2.0])
ax.set_xlabel('X')
ax.set_ylim3d([0.0, 2.0])
ax.set_ylabel('Y')
ax.set_zlim3d([0.0, 2.0])
ax.set_zlabel('Z')
ax.grid()

line, = ax.plot(np.array([0.]), np.array([0.]), np.array([0.]), lw=2.0)
anim = animation.FuncAnimation(fig, animate, init_func=init, frames=len(target_list), interval=20, blit=True)
plt.show()