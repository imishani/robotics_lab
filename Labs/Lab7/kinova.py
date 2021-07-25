#!/usr/bin/env python


import numpy as np
import sys
from transformations import quaternion_matrix
import copy
from scipy.spatial.transform import Rotation

import modern_robotics
import os

if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2
import signal
import sys
import time
import threading
import keyboard

class KinovaVS(object):
    '''
    A wrapper class for visual servoing related functions for kinova lite arm
    '''

    def __init__(self):

        self._t_hc, self._R_hc = np.array([0, 0, 0]), np.array([0, 0, 0, 0])

        self._R_hc = quaternion_matrix(self._R_hc)

        # frame transforms from camera to hand, only look up once
        self._T_hc = modern_robotics.RpToTrans(self._R_hc[:3, :3], self._t_hc)
        self._Ad_hc = modern_robotics.Adjoint(self._T_hc)

        # frame transforms from hand to body, look up in every loop
        self._T_bh = np.zeros((4, 4))
        self._Ad_bh = np.zeros((6, 6))

        if os.name != 'nt':
            settings = termios.tcgetattr(sys.stdin)
        try:
            # base_cyclic = main()
            sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
            import utilities

            # Parse arguments
            args = utilities.parseConnectionArguments()

            # Create connection to the device and get the router
            with utilities.DeviceConnection.createTcpConnection(args) as router:
                # Create required services
                self.base = BaseClient(router)
                self.base_cyclic = BaseCyclicClient(router)

        except KeyboardInterrupt:
            pass

    def update_hand_to_body_transforms(self):
        '''
        Update frame transforms for transformation matrix and twist
        '''

        try:
            feedback = self.base_cyclic.RefreshFeedback()

            t = [feedback.base.tool_pose_x,
                 feedback.base.tool_pose_y,
                 feedback.base.tool_pose_z]

            R = Rotation.from_euler('zyx', [feedback.base.tool_pose_theta_z,
                                            feedback.base.tool_pose_theta_y,
                                            feedback.base.tool_pose_theta_z], degrees=True)
        except :
            print('Warning! Cant get transformation from gripper to base.')

        R = quaternion_matrix(R)

        self._T_bh = modern_robotics.RpToTrans(R[:3, :3], t)  # Converts a rotation matrix and a position vector into homogeneous transformation matrix
        self._Ad_bh = modern_robotics.Adjoint(self._T_bh)

    def body_frame_twist(self, v_c):
        '''
        Take a 6x1 twist vector in camera frame,
        returns the Adjoint from camera frame to body frame,
        for calculating a camera frame twist in body frame
        Input: twist, [nu, omg], 1x6
        Output: twist, [nu, omg], 1x6
        '''

        self.update_hand_to_body_transforms()

        v_c = np.concatenate((v_c[3:6], v_c[0:3]))

        v_b = np.dot(self._Ad_bh, np.dot(self._Ad_hc, v_c))

        v_b = np.concatenate((v_b[3:6], v_b[0:3]))

        return v_b

    def set_joint_vel(self, vel_b):
        '''
        Takes a 6x1 twist vector in body frame,
        sets the corresponding joint velocities using the PyKDL package.
        Input: twist, [nu, omg], 1x6
        '''

        # Calculate joint velocities to achieve desired velocity
        # joint_vels = np.dot(self._kin.jacobian_pseudo_inverse(), vel_b)
        # joint_vels = np.array(joint_vels).reshape(-1, )
        #
        # joints = dict(zip(self._arm.joint_names(), joint_vels))
        #
        # # print "Joint Vel Command:{}".format(joint_vels)
        #
        # self._arm.set_joint_velocities(joints)

        self.example_twist_command(vel_b) # dont need to transform to each joint velocity

    def example_twist_command(self,vel):

        command = Base_pb2.TwistCommand()

        command.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_TOOL
        command.duration = 0

        twist = command.twist
        twist.linear_x = vel[0]
        twist.linear_y = vel[1]
        twist.linear_z = vel[2]
        twist.angular_x = vel[3]
        twist.angular_y = vel[4]
        twist.angular_z = vel[5]

        print("Sending the twist command for 5 seconds...")
        self.base.SendTwistCommand(command)

        # Let time for twist to be executed
        time.sleep(5)

        print("Stopping the robot...")
        self.base.Stop()
        time.sleep(1)

        return True

    def example_send_joint_speeds(self):

        SPEED = 20.0

        joint_speeds = Base_pb2.JointSpeeds()

        actuator_count = self.base.GetActuatorCount().count
        # The 7DOF robot will spin in the same direction for 10 seconds
        if actuator_count == 7:
            speeds = [SPEED, 0, -SPEED, 0, SPEED, 0, -SPEED]
            i = 0
            for speed in speeds:
                joint_speed = joint_speeds.joint_speeds.add()
                joint_speed.joint_identifier = i
                joint_speed.value = speed
                joint_speed.duration = 0
                i = i + 1
            print("Sending the joint speeds for 10 seconds...")
            self.base.SendJointSpeedsCommand(joint_speeds)
            time.sleep(10)

        # The 6 DOF robot will alternate between 4 spins, each for 2.5 seconds
        if actuator_count == 6:
            print("Sending the joint speeds for 10 seconds...")
            for times in range(4):
                del joint_speeds.joint_speeds[:]
                if times % 2:
                    speeds = [-SPEED, 0.0, 0.0, SPEED, 0.0, 0.0]
                else:
                    speeds = [SPEED, 0.0, 0.0, -SPEED, 0.0, 0.0]
                i = 0
                for speed in speeds:
                    joint_speed = joint_speeds.joint_speeds.add()
                    joint_speed.joint_identifier = i
                    joint_speed.value = speed
                    joint_speed.duration = 0
                    i = i + 1

                self.base.SendJointSpeedsCommand(joint_speeds)
                time.sleep(2.5)

        print("Stopping the robot")
        self.base.Stop()

        return True