# Import required packages


def path_planning(start, goal, obstacleList, area=[-0.05, 0.65], **args):
    """

    Args:
        start: start point (x_s, y_s) --> list: len=2 OR np.array(): shape=(2,)
        goal: end point (x_g, y_g) --> list: len=2 OR np.array(): shape=(2,)
        obstacleList: [(x_obs_1, y_obs_2, radius_obs_1), ..., (x_obs_N, y_obs_N, radius_obs_N)
        area: this is the area where you plan the path in. both x and y should be in between these boundaries.
        **args: add additional arguments as you please such as whether to plot a path or not.

    Returns:
        path: [[x_1, y_1], [x_2, y_2], ..., [x_M, y_M]] --> List of lists of size 2 (x, y).
                Important: Make sure that your output 'path' is in the right order (from start to goal)
    """3
    pass

def steering_angle(A_o_to_cam, A_c_to_cam, v_next_to_o):
    """

    Args:
        A_o_to_cam: Homogeneous matrix from origin to camera frame
        A_c_to_cam: Homogeneous matrix from car to camera frame
        v_next_to_o: 2d vector of v_next_to_c point in path with respect to origin frame: (x, y)

    Returns:
        v_next_to_c: 2d vector of v_next_to_c point in path with respect to car frame: (x, y)
        phi: Steering angle to v_next_to_c point.
    """
    pass
