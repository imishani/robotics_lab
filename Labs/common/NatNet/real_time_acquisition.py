from __future__ import print_function
import struct
import time
from NatNetClient import NatNetClient
from collections import defaultdict
from tqdm.auto import tqdm
import itertools
import re
import cv2
import pickle
import re
import os
from data_acquisition import save_data
from scipy.spatial.transform import Rotation as R
import numpy as np
import os
from os import system, name
import matplotlib.pyplot as plt
import pytransform3d.transformations as pytr
# from utile import save_img

def clear():
   # for windows
   if name == 'nt':
      _ = system('cls')


def quat_to_rot(Q):
    ROT = R.from_quat(Q).as_matrix()
    return ROT


def save_data(data, save_path, data_name='data_record'):
    t = str(time.asctime())
    file_name = data_name + " " + str(re.sub('[:!@#$]', '_', t) + '.pkl')
    file_name = file_name.replace(' ', '_')
    completeName = os.path.join(save_path, file_name)
    with open(completeName, "wb") as f:
        pickle.dump(data, f)
        del data


def save_img(path_img, name, i, image, ): # (img_name_save, img_name, i, frame)
    img_name = path_img + '/' + name + f'_{i}'
    tt = str(time.asctime())
    img_name_save = (img_name + " " + str(re.sub('[:!@#$]', '_', tt) + '.png')).replace(' ', '_')
    cv2.imwrite(img_name_save, image)
    return img_name_save


def receiveRigidBodyList(rigidBodyList, timestamp):
    for (ac_id, pos, quat, valid) in rigidBodyList:
        if not valid:
            continue


def countdown(time_factor):
    # Gives the user a few seconds to prepare for a
    # data recording session
    time.sleep(0.5)
    print(" ")
    print("The recording will begin in:")
    for i in range(0, 3):
        print(3 - i)
        time.sleep(time_factor)
    print(" ")

def countdownframe(time_factor):
    time.sleep(time_factor)


if __name__ == '__main__':
    start = 0.0
    ## Camera
    cam = cv2.VideoCapture(0)
    cv2.namedWindow("Received Data")
    W, H = 1920, 1080
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, W)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
    cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    cam.set(cv2.CAP_PROP_FPS, 30)

    ## Markers
    take_input = True
    reconnect = False
    save_path = r'C:\Users\User\Desktop\Eden\data\data_19_01\Eran\Angles'
    img_name_save = r'C:\Users\User\Desktop\Eden\data\data_19_01\Eran\image'

    # save_path = r'C:\Users\User\Desktop\Eden\data\data_5_1\Angles'
    # img_name_save = r'C:\Users\User\Desktop\Eden\data\data_5_1\Images'

    data_points_to_record = 2000
    data_points = 2
    to_plot = False
    record = False
    first_step = True

    marker_dict = defaultdict(list)
    keys = ['Finger_marker', 'Camera']
    Finger_marker = 3008
    Camera = 3007
    # This dictionary matches the rigid body id (key) to it's name (value)
    motive_matcher = {Finger_marker: 'Finger_marker',
                      Camera: 'Camera'}


    mistakes_dict = defaultdict(int)
    for key in keys:
        mistakes_dict[key] = 0

    times = []
    good_data = True
    error_start = time.time()
    temp_time = time.time()

    while True:
        try:
            natnet = NatNetClient(rigidBodyListListener=receiveRigidBodyList)
            natnet.run()
            break
        except struct.error:
            pass

    while True:
        if reconnect:
            while True:
                try:
                    natnet = NatNetClient(rigidBodyListListener=receiveRigidBodyList)
                    natnet.run()
                    break
                except struct.error:
                    pass

        if good_data:
            if time.time() - error_start > 1:
                record = True
                countdown(0.3)
                for i in tqdm(range(data_points_to_record)):
                        data_points += 1
                        times.append(time.time())

                        ### vision ###
                        ret, frame = cam.read()
                        marker_data = natnet.call()
                        if ret == True:
                            # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                            cv2.imshow('frame', frame)
                            img_name = 'Point'
                            id_name = save_img(img_name_save, img_name, i, frame)
                            if cv2.waitKey(1) & 0xFF == ord('q'):
                                break

                        ### Angles ###
                        assert marker_data[0][0] == Finger_marker
                        ROT_finger = quat_to_rot(marker_data[0][2])
                        A_finger = np.empty((4, 4))
                        A_finger[:3, :3] = ROT_finger
                        A_finger[:3, 3] = np.array(marker_data[0][1])
                        A_finger[3, :] = [0, 0, 0, 1]

                        assert marker_data[1][0] == Camera
                        ROT_camera = quat_to_rot(marker_data[1][2])
                        A_camera = np.empty((4, 4))
                        A_camera[:3, :3] = ROT_camera
                        A_camera[:3, 3] = np.array(marker_data[1][1])
                        A_camera[3, :] = [0, 0, 0, 1]
                        A_fc = np.matmul(np.linalg.inv(A_camera), A_finger)
                        xyz_norm = np.linalg.norm(A_fc[:3, 3])
                        x_unit = A_fc[:3, 0]
                        finger_pos = A_fc[:3,3]
                        theta_yaw = np.arctan2(x_unit[2], x_unit[0])
                        theta_pitch = np.arctan2(x_unit[1], np.linalg.norm([x_unit[0], x_unit[2]]))
                        print()
                        print('yaw_deg', np.rad2deg(theta_yaw))
                        print('pitch_deg', np.rad2deg(theta_pitch))
                        print('position', finger_pos)

                        for j, cell in enumerate(marker_data):
                            if cell[0] in list(motive_matcher.keys()):
                                key = motive_matcher[cell[0]]
                                data = {'position': cell[1], 'rotation': cell[2]}
                                if data['position'] == (0.0, 0.0, 0.0) and not first_step:
                                    mistakes_dict[key] += 1

                        marker_dict['theta_yaw'].append(np.rad2deg(theta_yaw))
                        marker_dict['pitch_deg'].append(np.rad2deg(theta_pitch))
                        marker_dict['finger_pose'].append(A_finger)
                        marker_dict['camera_pose'].append(A_camera)
                        marker_dict['finger_position'].append(finger_pos)
                        # marker_dict['id_name'].append(id_name)

                        if first_step:
                            start = time.time()
                            first_step = False

                        countdownframe(0.05)

                for key in keys:
                    if key in list(marker_dict.keys()):
                        del marker_dict[key]
                percentages = []
                for key in keys:
                    score = (1 - (data_points - mistakes_dict[key]) / data_points) * 100
                    percentages.append(score)
                if max(percentages) > 25:
                    for i, key in enumerate(keys):
                        print(f'The {key} element contains {percentages[i]:.1f}% missed frames.')
                    to_save = False
                    print("\n!!!!!!!!!! Too many missed markers, this recording will not be saved. !!!!!!!!!!\n")
                else:
                    to_save = True
                if to_save:
                    save_dict = {'marker_data': marker_dict,
                                 'time_stamps': times}
                    # if start >= 20:
                        # save_data(save_dict, save_path)
                cam.release()
                cv2.destroyAllWindows()
                start += 1

                break
        else:
            reconnect = True
            take_input = False
            natnet.stop()
