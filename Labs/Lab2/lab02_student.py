import numpy as np
from sympy import *
from scipy.spatial.transform import Rotation

alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha1:7')
a1, a2, a3, a4, a5, a6 = symbols('a1:7')
d1, d2, d3, d4, d5, d6 = symbols('d1:7')
q1, q2, q3, q4, q5, q6 = symbols('q1:7')


def set_dh_table():
    """

    Returns: dictionary

    Important: all length arguments in [m]
               all angles argument in [radians]

    """

    dh_subs_dict = {alpha1: , a1: , d1: , q1: q1 + ,
                    alpha2: , a2: , d2: , q2: q2 + ,
                    alpha3: , a3: , d3: , q3: q3 + ,
                    alpha4: , a4: , d4: , q4: q4 + ,
                    alpha5: , a5: , d5: , q5: q5 + ,
                    alpha6: , a6: , d6: , q6: q6 + }

    return dh_subs_dict


def dh(alpha, a, d, theta):
    """
    Returns:



    Args:
        alpha: torsion angle
        a: distance
        d: translation
        theta: rotation angle

    Returns: Homogeneous DH matrix

    Important note: use sympy cos/sin arguments instead of math/numpy versions.
    i.e: cos(theta) \ sin(theta)

    """

    return Matrix([[, , , ],
                   [, , , ],
                   [, , , ],
                   [, , , ]])

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
    pass

def xyz_rpy(A):
    """
    Extract translation and orientation in euler angles

    Args:
        A: Homogeneous transformation --> np.array((4, 4))

    Returns: x, y, z, roll, pitch, yaw --> np.array((1, 6))

    Important note: use numpy arrays

    """

    pass

def angles_to_follow():
    """

    Returns: Dictionary of the wanted angels

    """
    angles = { 't1': [, , , , , ],
               't2': [, , , , , ],
               't3': [, , , , , ],
               't4': [, , , , , ],
               't5': [, , , , , ]}  # [radians]


    return angles

