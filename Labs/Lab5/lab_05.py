#!/usr/bin/env python
import sys
import os

sys.path.insert(0, r'../common/Aruco_Tracker')
from aruco_module import aruco_track
from kinova import KinovaVS
from visual_servoing import PBVS
import cv2

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../common/robot"))
from robot_actions import *
from recorder import save
from scipy.spatial.transform import Rotation as R


if __name__ == '__main__':

    try:

        kinova_vs = KinovaVS()

        # setup connection with the kinova
        if os.name != 'nt':
            settings = termios.tcgetattr(sys.stdin)

        # Parse arguments
        args = utilities.parseConnectionArguments()

        # Create connection to the device and get the router
        with utilities.DeviceConnection.createTcpConnection(args) as router:
            # Create required services
            base = BaseClient(router)
            base_cyclic = BaseCyclicClient(router)

            controller = PBVS()
            controller._translation_only = False
            tracker = aruco_track()
            controller._lambda = 0.1
            vel_list, error_list, T_bh_list = [], [], []
            # set target pose

            try:
                """ Capture target frame
                    Set the desired transform we want to achieve from the marker"""
                track_transform = input('Press Enter to set the target frame ([tag])\n')
                t_target, R_target, _ = tracker.track()
                t_target, R_target = t_target.squeeze(), R_target.squeeze()
                R_target = R.from_rotvec(R_target).as_matrix()
            except:
                print('Error! Cannot find [tag] to [camera_frame] transform')
                sys.exit(0)

            controller.set_target_feature(t_target, R_target)

            while True:
                # get pose estimation from tracker node
                try:
                    t_curr, R_curr, _ = tracker.track()
                    t_curr, R_curr = t_curr.squeeze(), R_curr.squeeze()
                    R_curr = R.from_rotvec(R_curr).as_matrix()
                    vel_cam, error = controller.caculate_vel(t_curr, R_curr)
                    vel_body, vel_ee, T_bh = kinova_vs.body_frame_twist(vel_cam, base_cyclic)
                    print("Error: {:3.3f}, {:3.3f}, {:3.3f}, {:3.3f}, {:3.3f}, {:3.3f}".format(error[0], error[1],
                                                                                               error[2], error[3],
                                                                                               error[4], error[5]))
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break

                except:
                    print('Error! Cannot find [tag] to [camera_frame] transform')
                    vel_body = vel_cam = vel_ee = [0, 0, 0, 0, 0, 0]

                kinova_vs.set_joint_vel(vel_ee, base, base_cyclic)
                print("TCP Vel Command: {:3.3f}, {:3.3f}, {:3.3f}, {:3.3f}, {:3.3f}, {:3.3f}\n".format(vel_ee[0],
                                                                                                       vel_ee[1],
                                                                                                       vel_ee[2],
                                                                                                       vel_ee[3],
                                                                                                       vel_ee[4],
                                                                                                       vel_ee[5]))
                vel_list.append(vel_ee)
                error_list.append(error)
                T_bh_list.append(T_bh)

    except KeyboardInterrupt:
        save((vel_list, error_list, T_bh_list), prefix='5')
        pass
