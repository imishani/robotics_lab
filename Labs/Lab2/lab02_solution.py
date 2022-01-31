import numpy as np
from sympy import *
from scipy.spatial.transform import Rotation

alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha1:7')
a1, a2, a3, a4, a5, a6 = symbols('a1:7')
d1, d2, d3, d4, d5, d6 = symbols('d1:7')
q1, q2, q3, q4, q5, q6 = symbols('q1:7')


def set_dh_table():

    dh_subs_dict = {alpha1: pi / 2, a1: 0, d1: 0.1283 + 0.1150, q1: q1,
                    alpha2: pi, a2: 0.280, d2: 0.030, q2: q2 + pi / 2,
                    alpha3: pi / 2, a3: 0, d3: 0.020, q3: q3 + pi / 2,
                    alpha4: pi / 2, a4: 0, d4: 0.1400 + 0.1050, q4: q4 + pi / 2,
                    alpha5: pi / 2, a5: 0, d5: 0.0285 + 0.0285, q5: q5 + pi,
                    alpha6: 0, a6: 0, d6: 0.1050 + 0.130, q6: q6 + pi / 2}
    return dh_subs_dict

def dh(alpha, a, d, theta):
    """

    Args:
        alpha: torsion angle
        a: distance
        d: translation
        theta: rotation angle

    Returns: Homogeneous DH matrix

    Important note: use sympy cos/sin arguments

    """
    return Matrix([[cos(theta), -cos(alpha) * sin(theta), sin(theta) * sin(alpha), a * cos(theta)],
                     [sin(theta), cos(alpha) * cos(theta), -sin(alpha) * cos(theta), a * sin(theta)],
                     [0, sin(alpha), cos(alpha), d],
                     [0, 0, 0, 1]])


def FK():
    """

    Returns:
        End effector homogeneous matrix --> Matrix((4, 4))

    Hint 1:
            not like matrix multiplication in numpy where you should use "np.malmul" or "@" or "np.dot",
            here you need to use the simple "*" sign.
    Hint 2:
            Use the above functions you just wrote.

    """
    dictionary = set_dh_table()
    T_01 = dh(dictionary[alpha1], dictionary[a1], dictionary[d1], dictionary[q1])
    T_12 = dh(dictionary[alpha2], dictionary[a2], dictionary[d2], dictionary[q2])
    T_23 = dh(dictionary[alpha3], dictionary[a3], dictionary[d3], dictionary[q3])
    T_34 = dh(dictionary[alpha4], dictionary[a4], dictionary[d4], dictionary[q4])
    T_45 = dh(dictionary[alpha5], dictionary[a5], dictionary[d5], dictionary[q5])
    T_56 = dh(dictionary[alpha6], dictionary[a6], dictionary[d6], dictionary[q6])
    Tes = Matrix([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    T = T_01 * T_12 * T_23 * T_34 * T_45 * T_56 * Tes
    return T

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

    angles = {'t1': [0, 0, 0, 0, 0, 0],  # deg
                              't2': [0, 0, 0, 0, 0, pi/2], # pi / 2, 0, 0, pi / 4, pi / 4, pi / 4
                              't3': [0, np.deg2rad(344), np.deg2rad(75), 0, np.deg2rad(300), 0],
                              't4': [np.deg2rad(7), np.deg2rad(21), np.deg2rad(150), np.deg2rad(285), np.deg2rad(340),
                                     np.deg2rad(270)],
                              't5': [0, 0, 0, 0, 0, 0]}

    return angles

