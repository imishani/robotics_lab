"""
All right reserved to to Itamar Mishani and Osher Azulay
imishani@gmail.com, osherazulay@mail.tau.ac.il
"""

from RRT import RRT



"""
Import obstacle list in the following formate:

Example:

obstacleList = [(5, 5, 1), (3, 6, 2), (3, 8, 2), (3, 10, 2), (7, 5, 2),
                (9, 5, 2), (8, 10, 1)]  # [x, y, radius]
                
"""

# obstacleList = [(5, 5, 1), (3, 6, 2), (3, 8, 2), (3, 10, 2), (7, 5, 2),
#                 (9, 5, 2), (8, 10, 1)]
#
# # Input start and goal coordinates:
# goal = [6., 10.]   # [goal_x, goal_y]
# start = [0, 0]
#
# # If show_animation is True then animation will be plotted:
# show_animation=True
#
# # area of sampling:
# area = [-2, 15]     # min max random area for both x and y coordinates
#
#
#
# if __name__ == '__main__':
#     rrt = RRT(
#         start=start,
#         goal=goal,
#         rand_area=area,
#         obstacle_list=obstacleList)
#     path = rrt.planning(animation=show_animation)

def path_planning(start, goal, obstacleList, show_animation=False, area=[-1.1, 1.1]):
    rrt = RRT(
        start=start,
        goal=goal,
        rand_area=area,
        obstacle_list=obstacleList)
    path = rrt.planning(animation=show_animation)
    return path