#!/usr/bin/env python

import numpy as np
import transformations
from scipy.spatial.transform import Rotation as R
import modern_robotics

from lab05_solution import *

class VisualServoing(object):
    def __init__(self):
        self._translation_only = False
        self._lambda = 0.5
        self._target_features_set = False


class PBVS(VisualServoing):
    def __init__(self):
        super(PBVS, self).__init__()
        self._target_feature_t = []
        self._target_feature_R = []

    def set_target_feature(self, t_input, R_input):
        '''
        Set PBVS target feature
        Input:  (object in desired camera frame)
                t_input, 3x1 vector
                R_input, 3x3 matrix
        '''
        self._target_feature_t = np.array(t_input).flatten()
        self._target_feature_R = R_input
        self._target_features_set = True

        print("\n")
        print("PBVS Set Target:")
        print("t:{}".format(self._target_feature_t))
        print("R:{}".format(self._target_feature_R))

    def _calculate_error(self, t_input, R_input):
        '''
        Calculate error based on the input pose and the target pose
        Input:  (object in current camera frame)
                t_input, 1x3 vector
                R_input, 3x3 matrix
        Output: Error, [t_err, R_err], 6x1
        '''

        t_curr = np.array(t_input).flatten()
        R_curr = R_input

        return calculate_error(t_curr, R_curr, self._target_feature_t, self._target_feature_R)

        # # see paragraph above Eq.13
        # # of Chaumette, Francois, and Seth Hutchinson. "Visual servo control. I. Basic approaches."
        # # https://hal.inria.fr/inria-00350283/document
        #
        # t_del = t_curr - self._target_feature_t
        # R_del = np.dot(self._target_feature_R, R_curr.T)
        # R_del_homo = np.vstack((np.hstack((R_del, np.zeros((3, 1)))), np.array([0, 0, 0, 1])))
        # (theta, u, _) = transformations.rotation_from_matrix(R_del_homo)
        #
        # if self._translation_only:
        #     error = np.hstack((t_del, np.zeros(3)))
        # else:
        #     error = np.hstack((t_del, theta * u))

        # return error

    def _L(self, t_input, R_input):
        '''
        form interaction matrix / feature jacobian base on current camera pose
        Input:  (object in current camera frame)
                t_input, 1x3 vector
                R_input, 3x3 matrix
        Output: Interation Matrix (feature Jacobian), 6x6
        '''

        t_curr = np.array(t_input).flatten()
        R_curr = R_input

        return feature_jacobian(t_curr, R_curr, self._target_feature_R)

        # R_del = np.dot(self._target_feature_R, R_curr.T)
        # R_del_homo = np.vstack((np.hstack((R_del, np.zeros((3, 1)))), np.array([0, 0, 0, 1])))
        #
        # (theta, u, _) = transformations.rotation_from_matrix(R_del_homo)
        #
        # skew_symmetric = modern_robotics.VecToso3
        #
        # skew_t = skew_symmetric(t_curr)
        # L_theta_u = np.identity(3) - (theta / 2) * np.array(skew_symmetric(u)) + (
        #             1 - (np.sinc(theta) / ((np.sinc(theta / 2)) ** 2))) * np.dot(np.array(skew_symmetric(u)),
        #                                                                          np.array(skew_symmetric(u)))
        #
        # L_top = np.hstack((-np.identity(3), skew_t))
        # L_bottom = np.hstack((np.zeros((3, 3)), L_theta_u))
        #
        # L_out = np.vstack((L_top, L_bottom))
        #
        # return L_out

    def caculate_vel(self, t_input, R_input):
        '''
        calculate the twist of camera frame required to reach target pose
        Input:  (object in current camera frame)
                t_input, 1x3 vector
                R_input, 3x3 matrix
        Output: Twist in camera frame
                [nu_c, omg_c], 1x6
        '''

        L = self._L(t_input, R_input)

        error = self._calculate_error(t_input, R_input)

        return control(L, error, self._lambda), error

        # vel = -self._lambda * np.dot(np.linalg.pinv(L), error)

        # return vel, error