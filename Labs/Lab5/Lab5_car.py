"""

All right reserved to Itamar Mishani and Osher Azulay
imishani@gmail.com (or imishani@andrew.cmu.edu), osherazulay@mail.tau.ac.il

"""

from RRT import RRT
from matplotlib import pyplot as plt


def path_planning(start, goal, obstacleList, show_animation=False, area=[-0.05, 0.65]):
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
