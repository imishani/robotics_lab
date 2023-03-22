from __future__ import print_function
import struct
import pickle
import os
import time
from NatNetClient import NatNetClient
from collections import defaultdict
from tqdm.auto import tqdm
import itertools
import re
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
from termcolor import colored
import sys

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

if __name__ == '__main__':

    ## Camera
    cam = cv2.VideoCapture(0)
    cv2.namedWindow("Received Data")
    cam.set(3, 512)
    cam.set(4, 512)
    class_id = 1
    ## Markers
    take_input = True
    reconnect = False
    save_path = r'E:\IMU\eran_test\data'
    data_points_to_record = 1
    data_points = 1
    to_plot = False
    record = False
    first_step = True
    all_imu_raw_data = []

    marker_dict = defaultdict(list)
    frame_image = defaultdict(list)
    keys = ['Upper_foram', 'Lower_forarm', 'Finger_marker', 'Camera', 'body_marker']
    Upper_forarm = 1035
    Lower_forarm = 1034
    Finger_marker = 1032
    Camera = 1033
    body_marker = 1036
    # This dictionary matches the rigid body id (key) to it's name (value)
    motive_matcher = {Upper_forarm: 'Upper_foram',
                      Lower_forarm: 'Lower_forarm',
                      Finger_marker: 'Finger_marker',
                      Camera: 'Camera',
                      body_marker: 'body_marker'}

    mistakes_dict = defaultdict(int)
    for key in keys:
        mistakes_dict[key] = 0

    times = []
    good_data = True
    error_start = time.time()
    temp_time = time.time()
    countdown(0.9)
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
                for i in tqdm(range(data_points_to_record)):
                        data_points += 1
                        times.append(time.time())
                        # countdown(0.1)
                        # ### vision ###
                        ret, frame = cam.read()
                        if ret == True:
                            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                            cv2.imshow('frame', gray)
                            img_name_save = "E:\IMU\eran_test\images\opencv_frame_{}".format(i)
                            img_name = "opencv_frame_{}.png".format(i)
                            tt = str(time.asctime())
                            img_name_save = img_name_save + " " + str(re.sub('[:!@#$]', '_', tt) + '.png')
                            img_name_save = img_name_save.replace(' ', '_')
                            frame_image[class_id].append(gray)
                            cv2.imwrite(img_name_save, gray)
                            if cv2.waitKey(1) & 0xFF == ord('q'):
                                break

                        marker_data = natnet.call()
                        assert marker_data[0][0] == Finger_marker
                        ROT_finger = quat_to_rot(marker_data[0][2])
                        A_finger = np.empty((4, 4))
                        A_finger[:3, :3] = ROT_finger
                        A_finger[:3, 3] = np.array(marker_data[0][1])
                        A_finger[3, :] = [0, 0, 0, 1]

                        assert marker_data[3][0] == Camera
                        ROT_camera = quat_to_rot(marker_data[3][2])
                        A_camera = np.empty((4, 4))
                        A_camera[:3, :3] = ROT_camera
                        A_camera[:3, 3] = np.array(marker_data[3][1])
                        A_camera[3, :] = [0, 0, 0, 1]
                        A_fc = np.matmul(np.linalg.inv(A_camera), A_finger)
                        xyz_norm = np.linalg.norm(A_fc[:3, 3])


                        x_unit = A_fc[:3, 2]
                        theta_yaw = np.arctan2(x_unit[0], x_unit[2])
                        theta_pitch = np.arctan2(x_unit[1], np.linalg.norm([x_unit[0], x_unit[2]]))

                        print('norm:', xyz_norm)

                        sys.stdout.write(colored("\r" + f"yaw_deg : {np.rad2deg(theta_yaw)}", 'blue'))
                        sys.stdout.write(colored("\r" + f"pitch_deg : {np.rad2deg(theta_pitch)}", 'blue'))
                        sys.stdout.write(colored("\r" + f"position : {A_fc[:3,3]}", 'blue'))
                        sys.stdout.write(colored("\r" + f"norm : {xyz_norm}", 'blue'))


                        for j, cell in enumerate(marker_data):
                            if cell[0] in list(motive_matcher.keys()):
                                key = motive_matcher[cell[0]]
                                data = {'position': cell[1], 'rotation': cell[2]}
                                if data['position'] == (0.0, 0.0, 0.0) and not first_step:
                                    marker_dict[key + '_position'].append(marker_dict[key + '_position'][-1])
                                    marker_dict[key + '_position'].append(marker_dict[key + '_position'][-1])
                                    mistakes_dict[key] += 1
                                else:
                                    marker_dict[key + '_position'].append(data['position'])
                                    marker_dict[key + '_rotation'].append(data['rotation'])
                        marker_dict['theta_yaw'].append(np.rad2deg(theta_yaw))
                        marker_dict['pitch_deg'].append(np.rad2deg(theta_pitch))
                        if first_step:
                            start = time.time()
                            first_step = False
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
                    # save_dict = {'marker_data': marker_dict,
                    #              'vision_data': frame_image,
                    #              'time_stamps': times}
                    pass
                    # save_data(save_dict, save_path)
                cam.release()
                cv2.destroyAllWindows()
                break
        else:
            reconnect = True
            take_input = False
            natnet.stop()





