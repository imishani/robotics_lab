import numpy as np
import transformations
from scipy.spatial.transform import Rotation as R
import modern_robotics


def calculate_error(t_curr, R_curr, target_feature_t, target_feature_R, translation_only=False):
    '''
    Calculate error based on the input pose and the target pose
    Input:  (object in current camera frame)
            t_input, 1x3 vector
            R_input, 3x3 matrix
    Output: Error, [t_err, R_err], 6x1
    '''



    # see paragraph above Eq.13
    # of Chaumette, Francois, and Seth Hutchinson. "Visual servo control. I. Basic approaches."
    # https://hal.inria.fr/inria-00350283/document

    t_del = t_curr - target_feature_t
    R_del = np.dot(target_feature_R, R_curr.T)
    R_del_homo = np.vstack((np.hstack((R_del, np.zeros((3, 1)))), np.array([0, 0, 0, 1])))
    (theta, u, _) = transformations.rotation_from_matrix(R_del_homo)

    if translation_only:
        error = np.hstack((t_del, np.zeros(3)))
    else:
        error = np.hstack((t_del, theta * u))

    return error


def feature_jacobian(t_curr, R_curr, target_feature_R):
    '''
    form interaction matrix / feature jacobian base on current camera pose
    Input:  (object in current camera frame)
            t_input, 1x3 vector
            R_input, 3x3 matrix
    Output: Interation Matrix (feature Jacobian), 6x6
    '''


    R_del = np.dot(target_feature_R, R_curr.T)
    R_del_homo = np.vstack((np.hstack((R_del, np.zeros((3, 1)))), np.array([0, 0, 0, 1])))

    (theta, u, _) = transformations.rotation_from_matrix(R_del_homo)

    skew_symmetric = modern_robotics.VecToso3

    skew_t = skew_symmetric(t_curr)
    L_theta_u = np.identity(3) - (theta / 2) * np.array(skew_symmetric(u)) + (
                1 - (np.sinc(theta) / ((np.sinc(theta / 2)) ** 2))) * np.dot(np.array(skew_symmetric(u)),
                                                                             np.array(skew_symmetric(u)))

    L_top = np.hstack((-np.identity(3), skew_t))
    L_bottom = np.hstack((np.zeros((3, 3)), L_theta_u))

    L_out = np.vstack((L_top, L_bottom))

    return L_out

def control(L, error, _lambda):
    '''
    calculate the twist of camera frame required to reach target pose
    Input:  (object in current camera frame)
            t_input, 1x3 vector
            R_input, 3x3 matrix
    Output: Twist in camera frame
            [nu_c, omg_c], 1x6
    '''

    vel = -_lambda * np.dot(np.linalg.pinv(L), error)

    return vel