#!/usr/bin/env python
import sys
import os
sys.path.insert(0, r'../common/Aruco_Tracker-master')
from aruco_module import aruco_track
from kinova import KinovaVS
from visual_servoing import PBVS
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from scipy.spatial.transform import Rotation as R

if __name__ == '__main__':

    try:

        kinova_vs = KinovaVS()

        # setup connection with the kinova
        if os.name != 'nt':
            settings = termios.tcgetattr(sys.stdin)

        sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
        import utilities

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
            # set target pose

            try:
                """ Capture target frame
                    Set the desired transform we want to achieve from the marker"""
                track_transform = input('Press Enter to the target frame ([tag_0])\n')
                t_target, R_target = tracker.track()
                t_target, R_target = t_target.squeeze(), R_target.squeeze()
                R_target = R.from_rotvec(R_target).as_quat()
            except:
                print('Error! Cannot find [tag_0] to [desired_camera_frame] transform')
                sys.exit(0)

            controller.set_target_feature(t_target, R_target)

            # set up the kinova

            while True:  # TODO: add safety condition
                # get pose estimation from tracker node
                try:
                    t_curr, R_curr = tracker.track()
                    t_curr, R_curr = t_curr.squeeze(), R_curr.squeeze()
                    R_curr = R.from_rotvec(R_curr).as_quat()
                    vel_cam = controller.caculate_vel(t_curr, R_curr)  # calc vel w.r.t camera frame
                    vel_body, vel_ee = kinova_vs.body_frame_twist(vel_cam, base_cyclic) # convert cam_vel to vel w.r.t body frame
                except:
                    print('Error! Cannot find [tag_0] to [desired_camera_frame] transform')
                    vel_body = vel_cam = vel_ee = [0, 0, 0, 0, 0, 0]


                kinova_vs.set_joint_vel(vel_cam, base, base_cyclic)

                # if np.linalg.norm(vel_body) < 10:
                #     print("Stopping the robot")
                #     base.Stop()
                #     break

    except KeyboardInterrupt:
        pass
