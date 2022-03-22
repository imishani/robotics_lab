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

    # dh_subs_dict = {alpha1: , a1: , d1: , q1: q1 + ,
    #                 alpha2: , a2: , d2: , q2: q2 + ,
    #                 alpha3: , a3: , d3: , q3: q3 + ,
    #                 alpha4: , a4: , d4: , q4: q4 + ,
    #                 alpha5: , a5: , d5: , q5: q5 + ,
    #                 alpha6: , a6: , d6: , q6: q6 + }
    #
    # return dh_subs_dict

    pass


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

    # return Matrix([[, , , ],
    #                [, , , ],
    #                [, , , ],
    #                [, , , ]])

    pass

def FK(theta_list):

    """
    Args:
        theta_list: joint angle vector ---> list [1,6]
    Returns:
        End effector homogeneous matrix --> Matrix((4, 4))

    Hints:
        - we have added a sympy implementation with missing parts, u dont have to use the same method.
        - chain 'Tes' to T_06 at the end.
    """

    # T_01, T_12, T_23, T_34, T_45, T_56 = TODO
    # Tes = Matrix([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])  # mandatory
    # T = T_01 * T_12 * T_23 * T_34 * T_45 * T_56 * Tes

    ''' fill angles to dict for sympy calculations'''

    # theta_dict = {}
    # for i in range(len(theta_list)):
    #     theta_dict[q[i]] = theta_list[i]

    ''' 
    homogeneous transformation matrix from base_link to end_effector [type: numeric matrix] 
    because we are using sympy, we have to use evalf.
    '''
    # T_0G_eval = T.evalf(subs=theta_dict, chop=True, maxn=4)

    # return T_0G_eval

    pass

def xyz_euler(A):
    """
    Extract translation and orientation in euler angles

    Args:
        A: Homogeneous transformation --> np.array((4, 4))

    Returns: x, y, z, thetax, thetay, thetaz --> np.array((1, 6))

    Important note: use numpy arrays

    """

    pass

def angles_to_follow():
    """

    Returns: Dictionary of the desired angels

    """
    # angles = { 't1': [, , , , , ],
    #            't2': [, , , , , ],
    #            't3': [, , , , , ],
    #            't4': [, , , , , ],
    #            't5': [, , , , , ]}  # [radians]
    #
    #
    # return angles

    pass
