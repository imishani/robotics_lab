import numpy as np
from sympy import *
from scipy.spatial.transform import Rotation

alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha1:7')
a1, a2, a3, a4, a5, a6 = symbols('a1:7')
d1, d2, d3, d4, d5, d6 = symbols('d1:7')
q1, q2, q3, q4, q5, q6 = symbols('q1:7')


def set_dh_table():
    # dh_subs_dict = {alpha1: , a1: , d1: , q1: ,
    #                 alpha2: , a2: , d2: , q2: ,
    #                 alpha3: , a3: , d3: , q3: ,
    #                 alpha4: , a4: , d4: , q4: ,
    #                 alpha5: , a5: , d5: , q5: ,
    #                 alpha6: , a6: , d6: , q6: } # [mm - radians]

    dh_subs_dict = {alpha1: pi / 2, a1: 0, d1: 0.1283 + 0.1150, q1: q1,
                    alpha2: pi, a2: 0.280, d2: 0.030, q2: q2 + pi / 2,
                    alpha3: pi / 2, a3: 0, d3: 0.020, q3: q3 + pi / 2,
                    alpha4: pi / 2, a4: 0, d4: 0.1400 + 0.1050, q4: q4 + pi / 2,
                    alpha5: pi / 2, a5: 0, d5: 0.0285 + 0.0285, q5: q5 + pi,
                    alpha6: 0, a6: 0, d6: 0.1050 + 0.130, q6: q6 + pi / 2}
    return dh_subs_dict
    # pass

def dh(alpha, a, d, theta):
    """

    Args:
        alpha: torsion angle
        a: distance
        d: translation
        theta: rotation angle

    Returns: Homogenious DH matrix

    """
    return Matrix([[, , ,],
                 [, , , ],
                 [, , , ],
                 [, , , ]])
    pass

def FK(angles):
    """

    Args:
        angles: array of robot angles --> np.array((1, 6))

    Returns:
        End effector transformation matrix --> np.array((4, 4))

    """

    pass

def xyz_rpy(A):
    """

    Args:
        A: Homogenious transformation --> np.array((4, 4))

    Returns: x, y, z, roll, pitch, yaw --> np.array((1, 6))

    Important note: use numpy arrays

    """
    x = [np.array(A[0, -1]).astype(np.float64)]
    y = [np.array(A[1, -1]).astype(np.float64)]
    z = [np.array(A[2, -1]).astype(np.float64)]
    R = Rotation.from_matrix(A[:3, :3])
    Euler = R.as_rotvec()
    # Euler = R.as_euler('xyz', degrees=False)
    roll = [np.array(Euler[0]).astype(np.float64)]
    pitch = [np.array(Euler[1]).astype(np.float64)]
    yaw = [np.array(Euler[2]).astype(np.float64)]
    return np.array([x, y, z, roll, pitch, yaw])

    # pass

def angles_to_follow():
    """

    Returns: Dictionary of the wanted angels

    """
    # angles = { 't1': [, , , , , ],
    # 't2': [, , , , , ],
    # 't3': [, , , , , ],
    # 't4': [, , , , , ],
    # 't5': [, , , , , ]}  # [deg]

    angles = {'t1': [0, 0, 0, 0, 0, 0],  # deg
                              't2': [pi / 2, 0, 0, pi / 4, pi / 4, pi / 4],
                              't3': [0, np.deg2rad(344), np.deg2rad(75), 0, np.deg2rad(300), 0],
                              't4': [7, 21, 150, 285, 340, 270],
                              't5': [0, 0, 0, 0, 0, 0]}

    return angles