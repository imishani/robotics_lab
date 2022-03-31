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
from utils import Jacobian
import threading

class KinovaVS(object):
    '''
    A wrapper class for visual servoing related functions for kinova lite arm
    '''

    def __init__(self):

        self._t_hc, self._R_hc = np.array([0, 0, 0]), np.array([0, 0, 0, 1]) # TODO: EDIT!

        self._R_hc = quaternion_matrix(self._R_hc)

        # frame transforms from camera to hand, only look up once
        self._T_hc = modern_robotics.RpToTrans(self._R_hc[:3, :3], self._t_hc)
        self._Ad_hc = modern_robotics.Adjoint(self._T_hc)

        # frame transforms from hand to body, look up in every loop
        self._T_bh = np.zeros((4, 4))
        self._Ad_bh = np.zeros((6, 6))

    def update_hand_to_body_transforms(self, base_cyclic):
        '''
        Update frame transforms for transformation matrix and twist
        '''

        try:
            # get the transform from hand to body
            feedback = base_cyclic.RefreshFeedback()

            t = [feedback.base.tool_pose_x,
                 feedback.base.tool_pose_y,
                 feedback.base.tool_pose_z]

            r = Rotation.from_euler('zyx', [np.deg2rad(feedback.base.tool_pose_theta_z),
                                            np.deg2rad(feedback.base.tool_pose_theta_y),
                                            np.deg2rad(feedback.base.tool_pose_theta_x)], degrees=False)

            self._T_bh = modern_robotics.RpToTrans(r.as_matrix()[:3, :3], t)
            self._Ad_bh = modern_robotics.Adjoint(self._T_bh)
        except :
            print('Warning! Cant get transformation from gripper to base.')

    def body_frame_twist(self, v_c, base_cyclic):
        '''
        Take a 6x1 twist vector in camera frame,
        returns the Adjoint from camera frame to body frame,
        for calculating a camera frame twist in body frame
        Input: twist, [nu, omg], 1x6
        Output: twist, [nu, omg], 1x6
        '''

        self.update_hand_to_body_transforms(base_cyclic)

        v_c = np.concatenate((v_c[3:6], v_c[0:3]))

        """
        Get the transformation between the camera to the base, np.dot(self._Ad_bh, self._Ad_hc) 
                                                                       hand2body, cam2hand
        """

        v_b = np.dot(self._Ad_bh, np.dot(self._Ad_hc, v_c))

        v_b = np.concatenate((v_b[3:6], v_b[0:3]))

        return v_b

    def set_joint_vel(self, vel_b, base, base_cyclic):
        '''
        Takes a 6x1 twist vector in body frame,
        sets the corresponding joint velocities using the PyKDL package.
        Input: twist, [nu, omg], 1x6
        '''

        # Calculate joint velocities to achieve desired velocity
        actuator_count = base.GetActuatorCount().count
        Q = []
        for joint_id in range(actuator_count):
            Q.append(np.deg2rad(base_cyclic.RefreshFeedback().actuators[joint_id].position))

        joint_vels = np.dot(np.linalg.pinv(Jacobian(Q)), vel_b)
        joint_vels = np.array(joint_vels).reshape(-1, )

        print ("Joint Vel Command:{}".format(joint_vels))

        self.set_twist_command(joint_vels, base)

    def set_twist_command(self,vel, base):

        command = Base_pb2.TwistCommand()

        command.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_TOOL
        command.duration = 0

        twist = command.twist
        twist.linear_x = vel[0] * 0.1
        twist.linear_y = vel[1] * 0.1
        twist.linear_z = vel[2] * 0.1
        twist.angular_x = vel[3] * 0.1
        twist.angular_y = vel[4] * 0.1
        twist.angular_z = vel[5] * 0.1

        base.SendTwistCommand(command)

        time.sleep(0.01)


