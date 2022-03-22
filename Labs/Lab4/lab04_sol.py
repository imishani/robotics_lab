import numpy as np

def traj_gen_config(q1, q2, t, Tf):
    ''' path plan configuration space '''
    qm = q1 + (q2 - q1)/2
    a0 = q1
    a1 = np.zeros((6,))
    a4 = (qm - 0.5 * (q1 + q2)) / ((Tf / 2) ** 4)
    a3 = (2 * (q1 - q2) / (Tf ** 3)) - 2 * Tf * a4
    a2 = -1.5 * a3 * Tf - 2 * a4 * Tf ** 2

    q = a0 + a1 * t + a2 * t ** 2 + a3 * t ** 3 + a4 * t ** 4
    dq = a1 + 2 * a2 * t + 3 * a3 * t ** 2 + 4 * a4 * t ** 3
    ddq = 2 * a2 + 6 * a3 * t + 12 * a4 * t ** 2

    return q, dq, ddq

def traj_gen_task(x_s, x_g, t, Tf):

    """
    path plan in Task space
    x_s = Start point cartesian
    x_g = goal point cartesian
    """
    # x_s = np.array(list(x_s))   #start point
    # x_g = np.array(list(x_g))   #goal point
    a0 = 0. # np.zeros((3,))
    a1 = 0. # np.zeros((3,))
    a2 = 3 / Tf ** 2
    a3 = -2 / Tf ** 3
    x = x_s + (a0 + a1 * t + a2 * t ** 2 + a3 * t ** 3) * (x_g - x_s)
    dx = (a1 + 2 * a2 * t + 3 * a3 * t ** 2) * (x_g - x_s)
    ddx = (2 * a2 + 6 * a3 * t) * (x_g - x_s)

    return x, dx, ddx

def generate_x_goals_list():
    """

    Returns: list of

    """
    kTheta_x = 90.6
    kTheta_y = -1.0
    kTheta_z = 150.0
    x_ = np.array([[0.439, 0.194, 0.448, kTheta_x, kTheta_y, kTheta_z],
                   [0.200, 0.150, 0.400, kTheta_x, kTheta_y, kTheta_z],
                   [0.350, 0.050, 0.300, kTheta_x, kTheta_y, kTheta_z]])
    return x_


def generate_q_goals_list():
    """

    Returns: list of

    """
    jointPoses = np.array([[0.1, 343, 75, 354, 300, 0.1],
                           [7.5, 337, 80, 271, 287, 10],
                           [7.5, 313, 97, 272, 329, 10],
                           [0.1, 343, 75, 354, 300, 0.1]])

    return jointPoses