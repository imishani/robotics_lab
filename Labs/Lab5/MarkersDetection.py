import numpy as np
import time
import math
from scipy.spatial.transform import Rotation as R
import sys
sys.path.insert(0, r'../common/Aruco_Tracker-master')
from aruco_module import aruco_track
from Lab5_car import path_planning
from car_control import Controller
import cv2

car_ID = 4
axes_origin_ID = 16
s = time.time()


def calc_motor_command(angle):
    x = angle/180.
    if x >= 0:
        right = 1.
        left = -2*x + 1
    else:
        left = 1.
        right = 2*x + 1
    left = math.copysign(1, left) - left*0.5
    right = math.copysign(1, right) - right*0.5
    print(f'Left {left} right {right}')
    return left, right


def camera_data(t_curr, R_curr, ids, next_goal):
    try:
        t_curr, R_curr, ids = t_curr.squeeze(), R_curr.squeeze(), ids.squeeze()
        for i in range(len(ids)):
            trans[ids[i]] = t_curr[i, :]
            rot[ids[i]] = R.from_rotvec(R_curr[i, :]).as_matrix()
        err = trans[car_ID][:2] - trans[axes_origin_ID][:2] - np.array(next_goal)
        print(f'Error is: {err}')
        phi = np.rad2deg(np.arctan2(err[0], err[1]))
        return phi, err
    except:
        print('Error! Cannot detect frames')
        cntrlr.motor_command(1., 1.)
        # sys.exit(0)

if __name__ == "__main__":

    #Test
    tracker = aruco_track()
    car_ID = 4#int(input('Enter car ID:   '))
    goal_ID = 3#int(input('Enter goal ID:   '))
    cntrlr = Controller(car_ID)  # input car ID
    cntrlr.connect()
    time.sleep(1)
    cntrlr.motor_command(1., 1.)  # Don't move!
    t_curr, R_curr, ids = tracker.track()
    try:
        t_curr, R_curr, ids = t_curr.squeeze(), R_curr.squeeze(), ids.squeeze()
        trans, rot = {}, {}
        for i in range(len(ids)):
            trans[ids[i]] = t_curr[i, :]
            rot[ids[i]] = R.from_rotvec(R_curr[i, :]).as_matrix()
        # tracker.cap.release()
        # cv2.destroyAllWindows()
    except:
        print('Error! Cannot detect frames')
        sys.exit(0)

    ######## Plan a path:

    obs = ids[[i for i in range(len(ids)) if ids[i] not in [goal_ID, car_ID]]]
    obs = [np.hstack((trans[i][:2] - trans[axes_origin_ID][:2], 0.08)).tolist() for i in obs]
    path_ = path_planning((trans[car_ID][:2] - trans[axes_origin_ID][:2]).tolist(),
                          (trans[goal_ID][:2] - trans[axes_origin_ID][:2]).tolist(),
                          obstacleList=obs, show_animation=True)
    path_ = path_[::-1]

    ########
    # tracker = aruco_track()
    while cntrlr.Connected:
        tolerance = 0.1
        for next_goal in path_:
            err = 10e2
            while np.linalg.norm(err) > tolerance:
                t_curr, R_curr, ids = tracker.track()
                try:
                    phi, err = camera_data(t_curr, R_curr, ids, next_goal)
                except:
                    continue
                if np.linalg.norm(err) <= tolerance:
                    cntrlr.motor_command(1, 1)
                    continue
                left, right = calc_motor_command(phi)
                cntrlr.motor_command(-left, -right)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            cntrlr.motor_command(1, 1)

        print("Reached goal!! ")
        cntrlr.motor_command(1., 1.)
        tracker.cap.release()
        cv2.destroyAllWindows()
        sys.exit(0)

