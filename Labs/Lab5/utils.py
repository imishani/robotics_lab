import numpy as np
from sympy import *
from scipy.spatial.transform import Rotation

alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha1:7')
a1, a2, a3, a4, a5, a6 = symbols('a1:7')
d1, d2, d3, d4, d5, d6 = symbols('d1:7')
q1, q2, q3, q4, q5, q6 = symbols('q1:7')
q = (q1, q2, q3, q4, q5, q6)

def TF_matrix(alpha, a, d, q):
    TF = Matrix([[cos(q), -cos(alpha) * sin(q), sin(q) * sin(alpha), a * cos(q)],
                 [sin(q), cos(alpha) * cos(q), -sin(alpha) * cos(q), a * sin(q)],
                 [0, sin(alpha), cos(alpha), d],
                 [0, 0, 0, 1]])
    return TF

def set_dh_table():

    dh_subs_dict = {alpha1: pi / 2, a1: 0, d1: 0.1283 + 0.1150, q1: q1,
                    alpha2: pi, a2: 0.280, d2: 0.030, q2: q2 + pi / 2,
                    alpha3: pi / 2, a3: 0, d3: 0.020, q3: q3 + pi / 2,
                    alpha4: pi / 2, a4: 0, d4: 0.1400 + 0.1050, q4: q4 + pi / 2,
                    alpha5: pi / 2, a5: 0, d5: 0.0285 + 0.0285, q5: q5 + pi,
                    alpha6: 0, a6: 0, d6: 0.1050 + 0.130, q6: q6 + pi / 2}
    return dh_subs_dict

def set_tranform_matrices():
    tf_matrices_list = []
    dh_params = set_dh_table()
    T_01 = TF_matrix(alpha1, a1, d1, q1).subs(dh_params)
    tf_matrices_list.append(T_01)
    T_12 = TF_matrix(alpha2, a2, d2, q2).subs(dh_params)
    tf_matrices_list.append(T_01 * T_12)
    T_23 = TF_matrix(alpha3, a3, d3, q3).subs(dh_params)
    tf_matrices_list.append(T_01 * T_12 * T_23)
    T_34 = TF_matrix(alpha4, a4, d4, q4).subs(dh_params)
    tf_matrices_list.append(T_01 * T_12 * T_23 * T_34)
    T_45 = TF_matrix(alpha5, a5, d5, q5).subs(dh_params)
    tf_matrices_list.append(T_01 * T_12 * T_23 * T_34 * T_45)
    T_56 = TF_matrix(alpha6, a6, d6, q6).subs(dh_params)
    tf_matrices_list.append(T_01 * T_12 * T_23 * T_34 * T_45 * T_56)
    Tes = Matrix([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    T = T_01 * T_12 * T_23 * T_34 * T_45 * T_56 * Tes
    tf_matrices_list.append(T)
    return tf_matrices_list

def forward_hom_mat(theta_list):

    theta_dict = {}
    tf_matrices_list = set_tranform_matrices()
    T_0G = tf_matrices_list[-1]

    for i in range(len(theta_list)):
        theta_dict[q[i]] = theta_list[i]

    return T_0G.evalf(subs=theta_dict, chop=True, maxn=4)

def create_jacobian_syms():

    tf_matrices_list = set_tranform_matrices()

    T_0G = tf_matrices_list[-1]  # Get homogeneous transformation between base to TCP
    jacobian_mat = [diff(T_0G[:3, -1], q[i]).reshape(1, 3) for i in range(len(q))]
    jacobian_mat = Matrix(jacobian_mat).T
    temp = Matrix([0, 0, 1])
    for index in range(len(tf_matrices_list) - 2):
        temp = temp.col_insert(index + 1, tf_matrices_list[index][:3, 2])

    return Matrix(BlockMatrix([[jacobian_mat], [temp]]))

def LinearJacobian(Q):

    jacobian_mat_syms = create_jacobian_syms()
    return np.matrix(jacobian_mat_syms.evalf(subs=Q,                   # Get only the linear Jacobian
                                             chop=True,
                                             maxn=4)).astype(np.float64)[:3, :]

def Jacobian(Q):

    jacobian_mat_syms = create_jacobian_syms()
    theta_dict = {}
    for i in range(len(Q)):
        theta_dict[q[i]] = Q[i]
    return np.matrix(jacobian_mat_syms.evalf(subs=theta_dict,
                                             chop=True,
                                             maxn=4)).astype(np.float64)