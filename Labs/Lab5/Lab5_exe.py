import matplotlib.pyplot as plt
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
from tqdm import tqdm

axes_origin_ID = 16
s = time.time()


def calc_motor_command(angle):
    x = angle / 180.
    if x <= 0:
        right = 1.
        left = -2 * x + 1
    else:
        left = 1.
        right = 2 * x + 1
    left = math.copysign(1, left) - left * 0.5
    right = math.copysign(1, right) - right * 0.5
    # print(f'Left {left} right {right}')
    return left, right


def camera_data(t_curr, R_curr, ids, next_goal):
    try:
        t_curr, R_curr, ids = t_curr.squeeze(), R_curr.squeeze(), ids.squeeze()
        for i in range(len(ids)):
            trans[ids[i]] = t_curr[i, :]
            rot[ids[i]] = R.from_rotvec(R_curr[i, :]).as_matrix()
            homo[ids[i]] = np.vstack((np.hstack((rot[ids[i]], trans[ids[i]].reshape(-1, 1))), np.array([[0, 0, 0, 1]])))
        next = np.dot(np.linalg.inv(homo[4]) @ homo[16], np.hstack((np.array(next_goal)[::-1], np.array([0, 1]))).T)
        phi = np.rad2deg(np.arctan2(next[0], next[1]))
        # print(f'phi is: {phi}')
        # print(f'next goal is: {next}')
        plt.plot((np.linalg.inv(homo[axes_origin_ID]) @ homo[car_ID])[1, 3],
                 (np.linalg.inv(homo[axes_origin_ID]) @ homo[car_ID])[0, 3], 'ko', ms=2., alpha=0.4)
        plt.draw()
        return phi, next
    except:
        print('Error! Cannot detect frames')
        cntrlr.motor_command(1., 1.)
        # sys.exit(0)


if __name__ == "__main__":

    tracker = aruco_track()
    car_ID = int(input('Enter car ID:   '))
    goal_ID = int(input('Enter goal ID:   '))
    cntrlr = Controller(car_ID)  # input car ID
    cntrlr.connect()
    time.sleep(1)
    cntrlr.motor_command(1., 1.)  # Don't move!
    t_curr, R_curr, ids = tracker.track()
    try:
        t_curr, R_curr, ids = t_curr.squeeze(), R_curr.squeeze(), ids.squeeze()
        trans, rot, homo = {}, {}, {}
        for i in range(len(ids)):
            trans[ids[i]] = t_curr[i, :]
            rot[ids[i]] = R.from_rotvec(R_curr[i, :]).as_matrix()
            homo[ids[i]] = np.vstack((np.hstack((rot[ids[i]], trans[ids[i]].reshape(-1, 1))), np.array([[0, 0, 0, 1]])))
    except:
        print('Error! Cannot detect frames')
        sys.exit(0)

    # Plan a path:

    obs = ids[[i for i in range(len(ids)) if ids[i] not in [goal_ID, car_ID, axes_origin_ID]]]
    obs = [np.hstack((homo[i][:2, 3] - homo[axes_origin_ID][:2, 3], 0.06)).tolist() for i in obs]
    path_ = path_planning((homo[car_ID][:2, 3] - homo[axes_origin_ID][:2, 3]).tolist(),
                          (homo[goal_ID][:2, 3] - homo[axes_origin_ID][:2, 3]).tolist(),
                          obstacleList=obs, show_animation=True)
    path_ = path_[::-1][1:]

    # Apply CL plan tracking:
    while cntrlr.Connected:
        tolerance = 1.001
        i = 1
        for next_goal in path_:
            err = 10e2
            while np.linalg.norm(err) > tolerance:
                print(f'Attempting to reach point: {i} of {len(path_)}')
                t_curr, R_curr, ids = tracker.track()
                try:
                    phi, err = camera_data(t_curr, R_curr, ids, next_goal)
                    print(f' Phi: {round(phi)}, error: {err[:2]}\n')
                except:
                    continue
                if np.linalg.norm(err) <= tolerance:
                    cntrlr.motor_command(1, 1)
                    continue
                left, right = calc_motor_command(phi)
                cntrlr.motor_command(-left, -right)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            i += 1
            cntrlr.motor_command(1, 1)

        print("Reached goal!! ")
        cntrlr.motor_command(1., 1.)
        tracker.cap.release()
        cv2.destroyAllWindows()
        sys.exit(0)
