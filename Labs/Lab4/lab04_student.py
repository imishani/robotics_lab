import numpy as np


#######################
######## Lab 4 ########
#######################

def traj_gen_config(q1, q2, t, Tf):
    """
    path plan configuration space

    Args:
        q1: Start configuration (angles) [degrees] --> np.array((6,))
        q2: Goal configuration (angles) [degrees] --> np.array((6,))
        t: Time instance --> float
        Tf: Total movement time --> float

    Returns: angles positions, angles velocities, angles accelerations
                --> q, dq, ddq --> 3 object of np.array((6,))

    """
    pass


def traj_gen_task(x_s, x_g, t, Tf):
    """
    path plan in Task space

    Args:
        x_s: Start end-effector position and orientation UNITS:[m, degrees] --> np.array((6,))
        x_g: Goal end-effector position and orientation UNITS:[m, degrees] --> np.array((6,))
        t: Time instance --> float
        Tf: Total movement time --> float

    Returns: End-effector position, velocity, acceleration
                --> x, dx, ddx --> 3 object of np.array((6,))

    """

    pass


def generate_x_goals_list():
    """

    Returns: Desired end-effector goals along the planned path --> np.array((N, 6))

    Notes:  1. Position units [m]
            2. Orientation units [degrees]

    """
    pass


def generate_q_goals_list():
    """

    Returns: Desired configuration (angle) goals along the planned path --> --> np.array((N, 6))

    Notes: Orientation units [degrees]

    """

    pass
