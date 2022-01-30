#!/usr/bin/env python

import sys
import numpy as np

# from tf.transformations import *
# import tf
import sys
import os
sys.path.insert(0, r'C:\Users\RobLab\Desktop\robotics_lab\robotics_lab\Labs\common\Aruco_Tracker-master')
from aruco_module import aruco_track
from kinova import KinovaVS
from visual_servoing import PBVS


if __name__ == '__main__':

    try:

        kinova_vs = KinovaVS()
        controller = PBVS()
        controller._translation_only = True
        tracker = aruco_track()
        # set target pose

        try:
            """ Capture target frame
                Set the desired transform we want to achive from the marker"""
            track_transform = input('Press Enter to the target frame ([tag_0])\n')
            t_target, R_target = tracker.track()
        except:
            print('Error! Cannot find [tag_0] to [desired_camera_frame] transform')
            sys.exit(0)

        controller.set_target_feature(t_target, R_target)
        # set up the kinova

        while (True):  # TODO: add safety conditon
            # get pose estimation from tracker node
            try:
                t_curr, R_curr = tracker.track()
            except:
                print('Error! Cannot find [tag_0] to [desired_camera_frame] transform')
                continue

            # perform visual servoing
            vel_cam = controller.caculate_vel(t_curr, R_curr)

            vel_body = kinova_vs.body_frame_twist(vel_cam)

            kinova_vs.set_joint_vel(vel_body)

            if np.linalg.norm(vel_body) < 3:
                print("Stopping the robot")
                kinova_vs.base.Stop()

    except KeyboardInterrupt:
        pass
