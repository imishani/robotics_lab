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

from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2
import time
from utils import Jacobian

class KinovaVS(object):
    '''
    A wrapper class for visual servoing related functions for kinova lite arm
    '''

    def __init__(self):

        # constant translation between tool to camera frame
        self._t_hc = np.array([0., -0.08, 0])
        self._R_hc = np.eye(3)

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

            r = Rotation.from_euler('xyz', [np.deg2rad(feedback.base.tool_pose_theta_x),
                                            np.deg2rad(feedback.base.tool_pose_theta_y),
                                            np.deg2rad(feedback.base.tool_pose_theta_z)], degrees=False)

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
        v_e = np.dot(self._Ad_hc, v_c)
        v_b = np.dot(self._Ad_bh, v_e)

        v_b = np.concatenate((v_b[3:6], v_b[0:3]))
        v_e = np.concatenate((v_e[3:6], v_e[0:3]))

        return v_b, v_e

    def set_joint_vel(self, vel_b, base, base_cyclic):
        '''
        Takes a 6x1 twist vector in body frame,
        sets the corresponding joint velocities using the PyKDL package.
        Input: twist, [nu, omg], 1x6
        '''

        # actuator_count = base.GetActuatorCount().count
        # Q = []
        # for joint_id in range(actuator_count):
        #     Q.append(base_cyclic.RefreshFeedback().actuators[joint_id].position)
        #
        # joint_vels = np.dot(np.linalg.pinv(Jacobian(Q)), vel_b)
        #
        # print ("Joint Vel Command:{}".format(joint_vels))

        ''' because we can also control w.r.t end effector, we dont transfer to joint vels.'''
        joint_vels = np.array(vel_b).reshape(-1, )

        self.set_twist_command(joint_vels, base)
        # self.set_joint_speed(joint_vels, base)

    def set_twist_command(self,vel, base):

        command = Base_pb2.TwistCommand()

        ''' Pay attention to the frame '''
        command.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_TOOL
        command.duration = 0

        twist = command.twist
        twist.linear_x = vel[0] * 1
        twist.linear_y = vel[1] * 1
        twist.linear_z = vel[2] * 1
        twist.angular_x = vel[3] * 50
        twist.angular_y = vel[4] * 50
        twist.angular_z = vel[5] * 50

        base.SendTwistCommand(command)

        time.sleep(0.01)


    def set_joint_speed(self,vel , base):

        joint_speeds = Base_pb2.JointSpeeds()

        actuator_count = base.GetActuatorCount().count
        # The 7DOF robot will spin in the same direction for 10 seconds
        i = 0
        for speed in vel:
            joint_speed = joint_speeds.joint_speeds.add()
            joint_speed.joint_identifier = i
            joint_speed.value = np.rad2deg(speed)
            joint_speed.duration = 0
            i = i + 1

        base.SendJointSpeedsCommand(joint_speeds)
        time.sleep(0.01)


    def feedback(self, base_cyclic):
        # cur_joint, cur_end_xyz, cur_end_euler = [], [],[]
        # for i in range(6):
        #     cur_joint.append(base_cyclic.RefreshFeedback().actuators[i].position)

        cur_end_xyz = [base_cyclic.RefreshFeedback().base.tool_pose_x,
                       base_cyclic.RefreshFeedback().base.tool_pose_y,
                       base_cyclic.RefreshFeedback().base.tool_pose_z]

        cur_end_euler = [base_cyclic.RefreshFeedback().base.tool_pose_theta_x,
                        base_cyclic.RefreshFeedback().base.tool_pose_theta_y,
                        base_cyclic.RefreshFeedback().base.tool_pose_theta_z]

        return cur_end_xyz, cur_end_euler