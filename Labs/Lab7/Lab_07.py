#!/usr/bin/env python

import sys
import numpy as np

# from tf.transformations import *
# import tf
import sys
import os
sys.path.insert(0, r'C:\Users\RobLab\Desktop\robotics_lab\robotics_lab\Labs\common\Aruco_Tracker-master') #os.path.join(os.path.dirname(__file__))

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
        try: # TODO: add methods which extract TF from relative frame to desired
            # tf_listener.waitForTransform('/desired_camera_frame', '/tag_0', rospy.Time(), rospy.Duration(4.0))
            # (t_target, R_target) = tf_listener.lookupTransform('/desired_camera_frame', '/tag_0', rospy.Time(0))
            track_transform = input('Press Enter to calibrate current position as reference frame:')
            t_target, R_target = tracker.track()

        except:
            print('Error! Cannot find [tag_0] to [desired_camera_frame] transform')
            sys.exit(0)

        controller.set_target_feature(t_target, R_target)

        # set up baxter
        while (True):

            # get pose estimation from apriltag node
            try:
                # tf_listener.waitForTransform('/' + limb + '_hand_camera', '/tag_0', rospy.Time(), rospy.Duration(4.0))
                # (t_curr, R_curr) = tf_listener.lookupTransform('/' + limb + '_hand_camera', '/tag_0', rospy.Time(0))
                t_curr, R_curr = tracker.track()

            except:
                print('Error! Cannot find [tag_0] to [desired_camera_frame] transform')
                continue

            # perform visual servoing
            vel_cam = controller.caculate_vel(t_curr, R_curr)

            vel_body = kinova_vs.body_frame_twist(vel_cam)

            kinova_vs.set_joint_vel(vel_body)

    except KeyboardInterrupt:
        pass