from scipy.linalg import logm
import matplotlib.pyplot as plt
import matplotlib
import sys, os

if os.name == 'nt':
    import msvcrt
else:
    import tty, termios

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../common/robot"))
from robot_actions import *
from lab03_solution import *

np.set_printoptions(precision=2, suppress=True, threshold=5)
init_printing()
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import utilities


def round_expr(expr, num_digits):
    return expr.xreplace({n: round(n, num_digits) for n in expr.atoms(Number)})


class RoboticArmLab03:

    def __init__(self, simplified=False):
        self.joints = 0
        self.alpha, self.a, self.q, self.d = None, None, None, None
        self.dh_params = {}
        self.tf_matrices_list = []
        self.solutions = [(x, y, z) for x in [0, 1] for y in [0, 1] for z in [0, 1]]
        self.current_pos = []

        self.l1 = 0.2433
        self.dd1 = 0.010
        self.l2 = 0.280
        self.l3 = 0.245
        self.dwe = 0.105 + 0.130
        if simplified:
            self.dwo = 0.0
        else:
            self.dwo = 0.0285 * 2
        self.joint_limit = np.deg2rad(np.array([154.1, 150., 150.1, 148.98, 145.0, 148.98]))

        #               alpha a d r
        self.alpha0, self.alpha1, self.alpha2, self.alpha3, self.alpha4, self.alpha5, self.alpha6 = symbols('alpha0:7')
        self.a1, self.a2, self.a3, self.a4, self.a5, self.a6 = symbols('a1:7')
        self.d1, self.d2, self.d3, self.d4, self.d5, self.d6 = symbols('d1:7')
        self.q1, self.q2, self.q3, self.q4, self.q5, self.q6 = symbols('q1:7')

        dh_subs_dict = {self.alpha1: pi / 2, self.a1: 0, self.d1: self.l1, self.q1: self.q1,
                        self.alpha2: pi, self.a2: self.l2, self.d2: 0.030, self.q2: self.q2 + pi / 2,
                        self.alpha3: pi / 2, self.a3: 0, self.d3: 0.020, self.q3: self.q3 + pi / 2,
                        self.alpha4: pi / 2, self.a4: 0, self.d4: self.l3, self.q4: self.q4 + pi / 2,
                        self.alpha5: pi / 2, self.a5: 0, self.d5: self.dwo, self.q5: self.q5 + pi,
                        self.alpha6: 0, self.a6: 0, self.d6: self.dwe, self.q6: self.q6 + pi / 2}

        self.set_dh_param_dict(dh_subs_dict)

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
        T_01 = self.TF_matrix(self.alpha1, self.a1, self.d1, self.q1).subs(self.dh_params)
        self.tf_matrices_list.append(T_01)
        T_12 = self.TF_matrix(self.alpha2, self.a2, self.d2, self.q2).subs(self.dh_params)
        self.tf_matrices_list.append(T_01 * T_12)
        T_23 = self.TF_matrix(self.alpha3, self.a3, self.d3, self.q3).subs(self.dh_params)
        self.tf_matrices_list.append(T_01 * T_12 * T_23)
        T_34 = self.TF_matrix(self.alpha4, self.a4, self.d4, self.q4).subs(self.dh_params)
        self.tf_matrices_list.append(T_01 * T_12 * T_23 * T_34)
        T_45 = self.TF_matrix(self.alpha5, self.a5, self.d5, self.q5).subs(self.dh_params)
        self.tf_matrices_list.append(T_01 * T_12 * T_23 * T_34 * T_45)
        T_56 = self.TF_matrix(self.alpha6, self.a6, self.d6, self.q6).subs(self.dh_params)
        self.tf_matrices_list.append(T_01 * T_12 * T_23 * T_34 * T_45 * T_56)
        # Tes = Matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]) # Conf 1
        Tes = Matrix([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])  # Conf 2 - as in Matlab
        T = T_01 * T_12 * T_23 * T_34 * T_45 * T_56 * Tes
        self.tf_matrices_list.append(T)

    def show_transform_matrices(self):
        print('Transform Matrices are: {}'.format(self.tf_matrices_list))

    def check_joint_limit(self, Q):
        for q, lim in zip(Q, self.joint_limit):
            if np.abs(q) > lim:
                return False
        return True

    def gen_angles(self, limits=None):
        if limits is None:
            return np.array([np.random.random() * 2 * lim - lim for lim in self.joint_limit])
        else:
            return np.array([np.random.random() * 2 * limits - limits for lim in self.joint_limit])

    @staticmethod
    def TF_matrix(alpha, a, d, q):
        TF = Matrix([[cos(q), -cos(alpha) * sin(q), sin(q) * sin(alpha), a * cos(q)],
                     [sin(q), cos(alpha) * cos(q), -sin(alpha) * cos(q), a * sin(q)],
                     [0, sin(alpha), cos(alpha), d],
                     [0, 0, 0, 1]])
        return TF

    def plot_arm(self, ax, theta_list, color='black'):
        theta_dict = {}
        for i in range(len(theta_list)):
            theta_dict[self.q[i]] = theta_list[i]

        xp, yp, zp = 0, 0, 0
        for i, T in enumerate(self.tf_matrices_list):
            if i == 1:
                continue
            T = T.evalf(subs=theta_dict, chop=True, maxn=4)
            x = np.array(T[0, -1]).astype(np.float64)
            y = np.array(T[1, -1]).astype(np.float64)
            z = np.array(T[2, -1]).astype(np.float64)
            ax.plot3D([xp, x], [yp, y], [zp, z], color)
            ax.plot3D([x], [y], [z], 'o', color=color)
            xp, yp, zp = x, y, z

    def jacobian_func(self):

        T_0G = self.tf_matrices_list[-1]
        self.jacobian_mat = [diff(T_0G[:3, -1], self.q[i]).reshape(1, 3) for i in range(len(self.q))]
        # self.jacobian_mat = np.vstack((self.jacobian_mat, ))
        self.jacobian_mat = Matrix(self.jacobian_mat).T
        # to_jac = [list(A[:3, 2]) for A in self.tf_matrices_list]
        temp = Matrix([0, 0, 1])
        for index in range(len(self.tf_matrices_list) - 2):
            temp = temp.col_insert(index + 1, self.tf_matrices_list[index][:3, 2])

        self.jacobian_mat = Matrix(BlockMatrix([[self.jacobian_mat], [temp]]))

    def jacobian_calc(self, q):

        theta_dict = {}
        for i in range(len(q)):
            theta_dict[self.q[i]] = q[i]

        Ja, Jl = [], []

        Ja.append(np.array([0, 0, 1]))
        Pee = np.array(self.tf_matrices_list[-1][:3, 3].evalf(
            subs=theta_dict, chop=True, maxn=4)).astype(np.float64).squeeze()

        Jl.append(np.cross(Pee, np.array([0, 0, 1])))
        for i, T in enumerate(self.tf_matrices_list[:-2]):
            z = T[:3, 2]
            z = np.array(z.evalf(subs=theta_dict, chop=True, maxn=4)).astype(np.float64).squeeze()
            Ja.append(z)

            ri = np.array(T[:3, 3].evalf(subs=theta_dict, chop=True, maxn=4)).astype(np.float64).squeeze()
            r = Pee - ri
            Jl.append(np.cross(r, z))

        Ja = np.array(Ja).T  # .reshape(3,-1)
        Jl = np.array(Jl).T  # .reshape(3,-1)

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

    def inverse_kinematics_simplified(self, target, sol_num):

        # Position
        W = np.array(target[:3, 3] - self.dwe * target[:3, 2], dtype=np.float64).reshape(1, -1)[0]

        d = np.sqrt(W[0] ** 2 + W[1] ** 2 + (W[2] - self.l1) ** 2 - self.dd1 ** 2)
        alpha = np.arctan2(W[2] - self.l1, np.real(np.sqrt(W[0] ** 2 + W[1] ** 2 - self.dd1 ** 2)))

        if self.solutions[sol_num][0]:  # right arm
            theta1 = np.arctan2(W[1], W[0]) + np.arcsin(self.dd1 / np.sqrt(W[1] ** 2 + W[0] ** 2))
            if self.solutions[sol_num][1]:  # Elbow down
                theta2 = -np.pi / 2 + alpha - np.arccos((self.l2 ** 2 + d ** 2 - self.l3 ** 2) / (2 * d * self.l2))
                theta3 = -np.arccos((d ** 2 - self.l2 ** 2 - self.l3 ** 2) / (2 * self.l2 * self.l3))
            else:  # Elbow up
                theta2 = -np.pi / 2 + alpha + np.arccos((self.l2 ** 2 + d ** 2 - self.l3 ** 2) / (2 * d * self.l2))
                theta3 = np.arccos((d ** 2 - self.l2 ** 2 - self.l3 ** 2) / (2 * self.l2 * self.l3))
        else:  # left arm
            theta1 = np.arctan2(W[1], W[0]) - np.arcsin(self.dd1 / np.sqrt(W[1] ** 2 + W[0] ** 2)) + np.pi
            if not self.solutions[sol_num][1]:  # Elbow up
                theta2 = np.pi / 2 - alpha - np.arccos((self.l2 ** 2 + d ** 2 - self.l3 ** 2) / (2 * d * self.l2))
                theta3 = -np.arccos((d ** 2 - self.l2 ** 2 - self.l3 ** 2) / (2 * self.l2 * self.l3))
            else:  # Elbow down
                theta2 = -3 * np.pi / 2 - alpha + np.arccos((self.l2 ** 2 + d ** 2 - self.l3 ** 2) / (2 * d * self.l2))
                theta3 = np.arccos((d ** 2 - self.l2 ** 2 - self.l3 ** 2) / (2 * self.l2 * self.l3))

        theta1 = np.real(theta1)
        theta2 = np.real(theta2)
        theta3 = np.real(theta3)
        q123 = np.array([theta1, theta2, theta3])
        q123 = np.array([angle - 2 * np.pi if angle > np.pi else angle for angle in q123])
        q123 = np.array([angle + 2 * np.pi if angle < -np.pi else angle for angle in q123])

        # Orientation
        R03 = self.tf_matrices_list[2][:3, :3]

        theta_list = list(q123)
        theta_dict = {}
        for i in range(len(theta_list)):
            theta_dict[self.q[i]] = theta_list[i]
        R03 = np.array(R03.evalf(subs=theta_dict, chop=True, maxn=4))

        R = R03.T.dot(
            target[:3, :3])  # .dot(np.array([[0, 1, 0],[-1, 0, 0],[0, 0, 1]])) # Uncomment if using conf. 2 of Tes
        R = np.array(R, dtype=np.float64)

        if R[2, 2] == 1.0:
            q456 = [np.arctan2(R[1, 0], R[1, 1]), 0, 0]
        else:
            if self.solutions[sol_num][2]:  # Negative theta
                q456 = [np.arctan2(-R[0, 2], R[1, 2]), np.arctan2(-np.sqrt(R[0, 2] ** 2 + R[1, 2] ** 2), R[2, 2]),
                        np.arctan2(-R[2, 0], -R[2, 1])]
            else:
                q456 = [np.arctan2(R[0, 2], -R[1, 2]), np.arctan2(np.sqrt(R[0, 2] ** 2 + R[1, 2] ** 2), R[2, 2]),
                        np.arctan2(R[2, 0], R[2, 1])]

        q456 = np.array([np.real(angle) - 2 * np.pi if angle > np.pi else np.real(angle) for angle in q456])
        q456 = np.array([np.real(angle) + 2 * np.pi if angle < -np.pi else np.real(angle) for angle in q456])

        q = np.concatenate((q123, q456), axis=0)

        return q

    def inverse_kinematics_iterative(self, guess, target):

        # Initial Guess - Joint Angles
        Q = guess

        # Init Jacobian
        self.jacobian_func()

        error_grad = []

        theta_dict = {}
        error_v = error_w = 100

        lr = 1.
        counter = 0
        while error_v > 0.03 or error_w > 0.02:

            for i in range(len(Q)):
                theta_dict[self.q[i]] = Q[i]

            # print(counter, np.rad2deg(Q))
            T_q = np.array(self.forward_hom_mat(Q)).astype(np.float64)
            T_sd = np.array(target).astype(np.float64)
            Te = np.linalg.inv(T_q).dot(T_sd)
            Te = logm(Te)

            R = Te[:3, :3]
            w = np.array([R[2, 1], R[0, 2], R[1, 0]])
            v = Te[:3, 3]

            Vb = np.concatenate((w, v), axis=0)

            J = np.matrix(self.jacobian_mat.evalf(subs=theta_dict, chop=True, maxn=4)).astype(np.float64)

            Q = Q + lr * np.linalg.pinv(J).dot(Vb)  # .T
            Q = np.array(Q).astype(np.float64)
            Q = Q[0]

            #     lr = 0.2
            error_v = np.linalg.norm(Vb[:3])
            error_w = np.linalg.norm(Vb[3:])
            print(error_v, error_w)  # error
            counter += 1
            print()

        # Q = np.array([np.real(angle)%(np.sign(angle)*2*np.pi) for angle in Q])
        Q = np.array([np.real(angle) - 2 * np.pi if angle > np.pi else np.real(angle) for angle in Q])
        return Q

    def inverse_kinematics_iterative_position_old(self, guess, p_d):

        Q = guess  # Initial Guess - Joint Angles
        self.jacobian_func()  # Init Jacobian
        theta_dict = {}
        error = 100

        lr = 0.05
        counter = 0
        while error > 1e-2:

            for i in range(len(Q)):
                theta_dict[self.q[i]] = Q[i]

            print(counter, np.rad2deg(Q))

            T = np.array(self.forward_hom_mat(Q)).astype(np.float64)  # Get transformation between base to EE
            p = T[:3, 3].reshape((3,))  # Extract Rotation matrix

            # J = np.matrix(self.jacobian_mat.evalf(subs=theta_dict,  # Get only the linear Jacobian
            #                                       chop=True,
            #                                       maxn=4)).astype(np.float64)[:3, :]

            J = Jacobian(theta_dict)
            Q = Q + lr * np.linalg.pinv(J).dot(p_d - p)  # Update Q
            Q = np.array(Q).astype(np.float64)
            Q = Q[0]

            error = np.linalg.norm(p_d - p)
            print('*', error)
            counter += 1
            print()

        Q = np.array([np.real(angle) % (np.sign(angle) * 2 * np.pi) for angle in Q])
        Q = np.array([np.real(angle) - 2 * np.pi if angle > np.pi else np.real(angle) for angle in Q])
        return Q

    def inverse_kinematics_iterative_position(self, guess, p_d):

        Q = IK_NR_position(guess, p_d)

        Q = np.array([np.real(angle) % (np.sign(angle) * 2 * np.pi) for angle in Q])
        Q = np.array([np.real(angle) - 2 * np.pi if angle > np.pi else np.real(angle) for angle in Q])

        return Q


def move_to_angle_conf(angle_conf_eval):
    args = utilities.parseConnectionArguments()
    with utilities.DeviceConnection.createTcpConnection(args) as router:
        # Create required services
        base = BaseClient(router)
        base_cyclic = BaseCyclicClient(router)
        input("Remove any objects near the arm and press Enter")
        for i in range(len(angle_conf_eval)):

            Q = angle_conf_eval['target']
            # Create connection to the device and get the router
            # Example core
            success = True
            flag = True
            display = True

            while flag and success:

                if display:
                    key = input("Press H to move the arm  to home position\n"
                                "Press A to move the arm to desired angular position: \n"
                                + str(np.round(Q.squeeze(), 3)) + '\n'
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
                    success &= example_angular_action_movement(base, base_cyclic, Q=Q)
                    if success:
                        print('Successfully moved to arm to desired angular action')
                        flag = False
                    else:
                        print('Huston, we have a problem, please call the instructor')
                if str(key) == 'q' or str(key) == 'Q':
                    break


if __name__ == '__main__':
    arm = RoboticArmLab03()
    arm.set_joints(6)

    # Create desired TCP position
    q = np.deg2rad(np.array([39, -40, 50, 10, 20, 10]))
    # q = arm.gen_angles(limits = np.deg2rad(10)))

    # TODO: fill here the desired transformation matrix
    Tsd = arm.forward_hom_mat(q)

    print("tcp homogeneous transformation matrix: ")
    pprint(round_expr(Tsd, 2))
    print()

    fig = plt.figure(0)
    ax = plt.axes(projection='3d')
    arm.plot_arm(ax, q)
    print("\nClose the figure if you want to continue\n")
    plt.show()

    ###########################################
    ###### Newton simplified solution #########
    ###########################################

    q0 = np.deg2rad(np.array([0, -10, 50, 20, 10, 5]))  # arm.gen_angles()
    Tsd = np.array(Tsd).astype(np.float64)
    qn = arm.inverse_kinematics_iterative_position(guess=q0, p_d=Tsd[:3, 3])

    # print('Iterative IK solution:', np.rad2deg(qn))
    pprint(round_expr(arm.forward_hom_mat(qn), 2))
    fig = plt.figure(0)
    ax = plt.axes(projection='3d')
    arm.plot_arm(ax, q)
    arm.plot_arm(ax, qn, 'blue')
    print("\nClose the figure if you want to continue\n")
    plt.show()

    move_to_angle_conf({'target': np.rad2deg(qn)})

####################################
###### Analytical solution #########
####################################

# Qik = []
# d = 1000.
# im = 0
# for i in range(8):
#     qik = arm.inverse_kinematics_simplified(Tsd, i)
#     print('Analytical IK solution #%d:'%i, np.rad2deg(qik))
#     Qik.append(np.copy(qik))
#     if np.linalg.norm(qik-q) < d:
#         im = i
#         d = np.linalg.norm(qik-q)
# qik = Qik[im]
# arm.plot_arm(ax, qik, 'red')

####################################
###### Newton solution #############
####################################

# q0 = q + np.deg2rad(np.random.random((1,6)) * 3. - 1.5)[0]
# qn = arm.inverse_kinematics_iterative(q0, Tsd)
# arm.plot_arm(ax, qn, 'blue')
# print('Original q:', np.rad2deg(q))
# print('Iterative IK solution:', np.rad2deg(qn))
# print('IK q:', np.rad2deg(qik))
# Q = []
# while len(Q) < 8:
#     q0 = arm.gen_angles()
#     qn = arm.inverse_kinematics_iterative(q0, Tsd)
#     D = [np.linalg.norm(qn-qp) for qp in Q]
#     if len(D) == 0 or np.all(np.array(D)>1e-2):
#         Q.append(qn)
#         arm.plot_arm(ax, qn, 'blue')
# print()
# for qn in Q:
#     print(np.rad2deg(qn))
#     print(arm.forward_hom_mat(qn))
# print('---')
# print('Original q:', np.rad2deg(q))
# Tsd = np.array(Tsd).astype(np.float64)
# print(Tsd)
