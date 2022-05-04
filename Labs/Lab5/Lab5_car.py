"""

All right reserved to Itamar Mishani and Osher Azulay
imishani@gmail.com (or imishani@andrew.cmu.edu), osherazulay@mail.tau.ac.il

"""

from RRT import RRT
from matplotlib import pyplot as plt
import numpy as np


def planner(start, goal, obstacleList, show_animation=False, area=[-0.05, 0.65]):
    rrt = RRT(
        start=start,
        goal=goal,
        rand_area=area,
        obstacle_list=obstacleList)
    path = rrt.planning(animation=show_animation)

    if path is None:
        print("Cannot find path")
        return None
    else:
        print("found path!!")
        # Draw final path
        if show_animation:
            rrt.draw_graph()
            plt.plot([x for (x, y) in path], [y for (x, y) in path], '-or', ms=4., alpha=0.5)
            plt.grid(True)
            plt.pause(0.01)  # Need for Mac
        return path[::-1]

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
    v_next_to_c = np.dot(np.linalg.inv(A_c_to_cam) @ A_o_to_cam,
                         np.hstack((np.array(v_next_to_o), np.array([0, 1]))).T)
    phi = np.rad2deg(np.arctan2(v_next_to_c[0], v_next_to_c[1]))
    return v_next_to_c, phi