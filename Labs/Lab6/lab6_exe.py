"""
All right reserved to  Itamar Mishani and Osher Azulay
imishani@gmail.com (or imishani@andrew.cmu.edu), osherazulay@mail.tau.ac.il
"""

import matplotlib.pyplot as plt
import math
from scipy.spatial.transform import Rotation as R
import sys, os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../common/car"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../common/robot"))

from recorder import *
from Labs.common.car.car_control import Controller

sys.path.insert(0, r'../common/Aruco_Tracker')
from aruco_module import aruco_track

import cv2

from lab6_car import planner, steering_angle

# from Lab6_student import planner, steering_angle

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
    return left, right


def camera_data(t_curr, R_curr, ids, next_goal):
    e = 0.1
    try:
        t_curr, R_curr, ids = t_curr.squeeze(), R_curr.squeeze(), ids.squeeze()
        for i in range(len(ids)):
            trans[ids[i]] = e * trans[ids[i]] + (1 - e) * t_curr[i, :]
            rot[ids[i]] = e * rot[ids[i]] + (1 - e) * R.from_rotvec(R_curr[i, :]).as_matrix()
            homo[ids[i]] = np.vstack((np.hstack((rot[ids[i]], trans[ids[i]].reshape(-1, 1))), np.array([[0, 0, 0, 1]])))
        v_next, phi = steering_angle(homo[axes_origin_ID], homo[car_ID], np.array(next_goal))
        curr_x, curr_y = (np.linalg.inv(homo[axes_origin_ID]) @ homo[car_ID])[0, 3], \
                         (np.linalg.inv(homo[axes_origin_ID]) @ homo[car_ID])[1, 3]
        plt.plot(curr_x, curr_y, 'ko', ms=2., alpha=0.4)
        plt.draw()
        return phi, v_next, (curr_x, curr_y)
    except:
        print('Error! Cannot detect frames')
        cntrlr.motor_command(1., 1.)
        # sys.exit(0)


if __name__ == "__main__":

    tracker = aruco_track()
    axes_origin_ID = 1  # int(input('Enter origin ID:   '))
    car_ID = 2  # int(input('Enter car ID:   '))
    goal_ID = 13  # int(input('Enter goal ID:   '))
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

    obs = ids[[i for i in range(len(ids)) if ids[i] not in [goal_ID, car_ID, axes_origin_ID]]]
    # obs = [np.hstack((homo[i][:2, 3] - homo[axes_origin_ID][:2, 3], 0.05)).tolist() for i in obs]
    obs = [np.hstack(((np.linalg.inv(homo[axes_origin_ID]) @ homo[i])[:2, 3], 0.05)).tolist() for i in obs]

    path_ = planner((np.linalg.inv(homo[axes_origin_ID]) @ homo[car_ID])[:2, 3].tolist(),
                    (np.linalg.inv(homo[axes_origin_ID]) @ homo[goal_ID])[:2, 3].tolist(),
                    obs, args=True)

    # Apply CL plan tracking:
    executed_path = []
    while cntrlr.Connected:
        tolerance = 0.015
        i = 1
        for next_goal in path_:
            print(f'Attempting to reach point: {i} of {len(path_)}')
            next_ = [10e2, 10e2]
            while np.linalg.norm(next_[:2]) > tolerance:
                t_curr, R_curr, ids = tracker.track()
                try:
                    phi, next_, curr = camera_data(t_curr, R_curr, ids, next_goal)
                    executed_path.append(list(curr))

                    print(f' Phi: {round(phi)}, error: {next_[:2]}\n Distance: {np.linalg.norm(next_[:2])}')
                except:
                    continue
                if np.linalg.norm(next_[:2]) <= tolerance:
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
        if input('Save data? (y,n)   ') == 'y':
            save([path_, executed_path], prefix='6')
            print('Saved data as list of size 2 where first element is planned path and second is executed path!')
        sys.exit(0)
