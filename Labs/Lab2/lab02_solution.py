import numpy as np
from sympy import *
from scipy.spatial.transform import Rotation

alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha1:7')
a1, a2, a3, a4, a5, a6 = symbols('a1:7')
d1, d2, d3, d4, d5, d6 = symbols('d1:7')
q1, q2, q3, q4, q5, q6 = symbols('q1:7')
q = (q1, q2, q3, q4, q5, q6)

def set_dh_table():
    """
    DH table
    Returns: dh dict
    Important: all length arguments in [m]
               all angles argument in [radians]
    """

    dh_subs_dict = {alpha1: pi / 2,    a1: 0,     d1: 0.1283 + 0.115,    q1: q1,
                    alpha2: pi,        a2: 0.28,  d2: 0.030,             q2: q2 + pi / 2,
                    alpha3: pi / 2,    a3: 0,     d3: 0.020,             q3: q3 + pi / 2,
                    alpha4: pi / 2,    a4: 0,     d4: 0.140 + 0.105,     q4: q4 + pi / 2,
                    alpha5: pi / 2,    a5: 0,     d5: 0.0285 + 0.0285,   q5: q5 + pi,
                    alpha6: 0,         a6: 0,     d6: 0.1050 + 0.130,    q6: q6 + pi / 2}

    return dh_subs_dict

def dh(alpha, a, d, theta):
    """

    Args:
        alpha: torsion angle
        a: distance
        d: translation
        theta: rotation angle

    Returns: Homogeneous DH matrix

    Important note: use sympy cos/sin arguments instead of math/numpy versions.
    i.e: cos(theta) \ sin(theta)
    """
    return Matrix([[cos(theta), -cos(alpha) * sin(theta), sin(theta) * sin(alpha), a * cos(theta)],
                   [sin(theta), cos(alpha) * cos(theta), -sin(alpha) * cos(theta), a * sin(theta)],
                   [0, sin(alpha), cos(alpha), d],
                   [0, 0, 0, 1]])

def transform_matrices():
    """
    Returns:
        End effector homogeneous matrix --> Matrix((4, 4))

    Hint 1:
            not like matrix multiplication in numpy where you should use "np.malmul" or "@" or "np.dot",
            here you need to use the simple "*" sign.
    Hint 2:
            Use the above functions you just wrote.

    """
    dh_dict = set_dh_table()

    T_01 = dh(dh_dict[alpha1], dh_dict[a1], dh_dict[d1], dh_dict[q1])
    T_12 = dh(dh_dict[alpha2], dh_dict[a2], dh_dict[d2], dh_dict[q2])
    T_23 = dh(dh_dict[alpha3], dh_dict[a3], dh_dict[d3], dh_dict[q3])
    T_34 = dh(dh_dict[alpha4], dh_dict[a4], dh_dict[d4], dh_dict[q4])
    T_45 = dh(dh_dict[alpha5], dh_dict[a5], dh_dict[d5], dh_dict[q5])
    T_56 = dh(dh_dict[alpha6], dh_dict[a6], dh_dict[d6], dh_dict[q6])
    Tes = Matrix([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    T = T_01 * T_12 * T_23 * T_34 * T_45 * T_56 * Tes

    return T

def FK(theta_list):
    """
    Args:
        theta_list: joint angle vector
    Returns:
        End effector homogeneous matrix --> Matrix((4, 4))

    Hint 1:
            not like matrix multiplication in numpy where you should use "np.malmul" or "@" or "np.dot",
            here you need to use the simple "*" sign.
    Hint 2:
            Use the above functions you just wrote.

    """

    dh_dict = set_dh_table()

    T_01 = dh(dh_dict[alpha1], dh_dict[a1], dh_dict[d1], dh_dict[q1])
    T_12 = dh(dh_dict[alpha2], dh_dict[a2], dh_dict[d2], dh_dict[q2])
    T_23 = dh(dh_dict[alpha3], dh_dict[a3], dh_dict[d3], dh_dict[q3])
    T_34 = dh(dh_dict[alpha4], dh_dict[a4], dh_dict[d4], dh_dict[q4])
    T_45 = dh(dh_dict[alpha5], dh_dict[a5], dh_dict[d5], dh_dict[q5])
    T_56 = dh(dh_dict[alpha6], dh_dict[a6], dh_dict[d6], dh_dict[q6])
    Tes = Matrix([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    T = T_01 * T_12 * T_23 * T_34 * T_45 * T_56 * Tes

    ''' fill angles to dict for sympy calculations'''
    theta_dict = {}
    for i in range(len(theta_list)):
        theta_dict[q[i]] = theta_list[i]

    '''homogeneous transformation matrix from base_link to end_effector [type: numeric matrix] '''

    T_0G_eval = T.evalf(subs=theta_dict, chop=True, maxn=4)

    return T_0G_eval

def xyz_rpy(A):

    """
    Extract translation and orientation in euler angles

    Args:
        A: Homogeneous transformation --> np.array((4, 4))

    Returns: x, y, z, roll, pitch, yaw --> np.array((1, 6))
             x, y, z --> [m]
             roll, pitch, yaw -- > [degrees]

    Important note: use numpy arrays

    """
    x = [np.array(A[0, -1]).astype(np.float64)]
    y = [np.array(A[1, -1]).astype(np.float64)]
    z = [np.array(A[2, -1]).astype(np.float64)]
    R = Rotation.from_matrix(A[:3, :3])
    Euler = R.as_rotvec(degrees=True)
    roll = [np.array(Euler[0]).astype(np.float64)]
    pitch = [np.array(Euler[1]).astype(np.float64)]
    yaw = [np.array(Euler[2]).astype(np.float64)]
    return np.array([x, y, z, roll, pitch, yaw])

def angles_to_follow():
    """

    Returns: Dictionary of wanted angels

    """

    angles = {'t1': [0, np.deg2rad(20), np.deg2rad(50), 0, np.deg2rad(70), 0],  # deg
              't2': [0, pi/2, 0, 0, 0, pi/2],
              't3': [0, np.deg2rad(344), np.deg2rad(75), 0, np.deg2rad(300), 0],
              't4': [np.deg2rad(7), np.deg2rad(21), np.deg2rad(150), np.deg2rad(285), np.deg2rad(340),
                     np.deg2rad(270)],
              't5': [0, 0, 0, 0, 0, 0]}

    return angles

