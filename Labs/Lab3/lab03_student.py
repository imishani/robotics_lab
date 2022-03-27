import numpy as np
from sympy import *
from scipy.spatial.transform import Rotation

alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha1:7')
a1, a2, a3, a4, a5, a6 = symbols('a1:7')
d1, d2, d3, d4, d5, d6 = symbols('d1:7')
q1, q2, q3, q4, q5, q6 = symbols('q1:7')

'''
Hint1: you should use your functions from the previous lab
Hint2: using sympy is easier for debugging, but not mandatory
'''

def Jacobian(Q):
    '''

    Args:
        Q: joint configuration list [1,6]

    Returns:
        Full Jacobian matrix [6,6]
    '''

    pass

def LinearJacobian(Q):
    '''

    Args:
        Q: joint configuration list [1,6]

    Returns:
        Linear part of the Jacobian matrix [3,6]
    '''

    pass

def IK_NR_position(guess, target):
    '''

    Args:
        guess: initial angle guess list [1,6] {q1-6}
        target: task configuration tcp position [1,3] {x,y,z}

    Returns:
        Q* - joint configuration angles [1, 6]
    '''
    pass