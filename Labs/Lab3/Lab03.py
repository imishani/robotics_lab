#! /usr/bin/env python3

###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2018 Kinova inc. All rights reserved.
#
# This software may be modified and distributed
# under the terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.
#
###
import numpy as np
import sys
import os
import time
import threading

import sys, select, os
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

# Maximum allowed waiting time during actions (in seconds)
TIMEOUT_DURATION = 20
e = """
Communications Failed
"""

def getKey():
    if os.name == 'nt':
        return msvcrt.getch()
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

# Create closure to set an event after an END or an ABORT
# Maximum allowed waiting time during actions (in seconds)
TIMEOUT_DURATION = 20


# Create closure to set an event after an END or an ABORT
def check_for_end_or_abort(e):
    """Return a closure checking for END or ABORT notifications

    Arguments:
    e -- event to signal when the action is completed
        (will be set when an END or ABORT occurs)
    """

    def check(notification, e=e):
        print("EVENT : " + \
              Base_pb2.ActionEvent.Name(notification.action_event))
        if notification.action_event == Base_pb2.ACTION_END \
                or notification.action_event == Base_pb2.ACTION_ABORT:
            e.set()

    return check


def example_angular_action_movement(base, base_cyclic, Q = None):
    print("Starting angular action movement ...\n Adding 10 deg to each motor")
    action = Base_pb2.Action()
    action.name = "Example angular action movement"
    action.application_data = ""
    ok = False
    actuator_count = base.GetActuatorCount()
    if Q:
        for joint_id in range(actuator_count):
            joint_angle = action.reach_joint_angles.joint_angles.joint_angles.add()
            joint_angle.value = Q[joint_id]
    else:
        while not ok:
            dtheta = float(input("Enter required angular delta for each of the motors"))
            if abs(dtheta) < 10:
                print('Input delta are ok, executing')
                ok = True
            else:
                print('Only less then 10 [deg] delta are valid.')

        for joint_id in range(actuator_count):
            joint_angle = action.reach_joint_angles.joint_angles.joint_angles.add()
            joint_angle.value = base_cyclic.RefreshFeedback().actuators[joint_id].position + dtheta

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    print("Executing action")
    base.ExecuteAction(action)

    print("Waiting for movement to finish ...")
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("Angular movement completed")
    else:
        print("Timeout on action notification wait")
    return finished


def example_cartesian_action_movement(base, base_cyclic, C = None):
    print("Starting Cartesian action movement ...")

    action = Base_pb2.Action()
    action.name = "Example Cartesian action movement"
    action.application_data = ""

    feedback = base_cyclic.RefreshFeedback()
    if C:
        cartesian_pose = action.reach_pose.target_pose
        cartesian_pose.x = C[0]  # (meters)
        cartesian_pose.y = C[1]  # (meters)
        cartesian_pose.z = C[2]  # (meters)
        cartesian_pose.theta_x = C[3]  # (degrees)
        cartesian_pose.theta_y = C[4]  # (degrees)
        cartesian_pose.theta_z = C[5]  # (degrees)
    else:
        ok = False
        while not ok:
            d_y = float(input("Enter required delta y for the end effector position"))
            d_x = float(input("Enter required delta x for the end effector position"))
            if abs(d_y) < 0.1 and abs(d_x) < 0.1:
                print('Input deltas are ok, executing')
                ok = True
            else:
                print('Only less then 0.1 [m] delta inputs are valid.')

        cartesian_pose = action.reach_pose.target_pose
        cartesian_pose.x = feedback.base.tool_pose_x  # (meters)
        cartesian_pose.y = feedback.base.tool_pose_y - d_y  # (meters)
        cartesian_pose.z = feedback.base.tool_pose_z - d_x  # (meters)
        cartesian_pose.theta_x = feedback.base.tool_pose_theta_x  # (degrees)
        cartesian_pose.theta_y = feedback.base.tool_pose_theta_y  # (degrees)
        cartesian_pose.theta_z = feedback.base.tool_pose_theta_z  # (degrees)

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    print("Executing action")
    base.ExecuteAction(action)

    print("Waiting for movement to finish ...")
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("Cartesian movement completed")
    else:
        print("Timeout on action notification wait")
    return finished

def example_move_to_home_position(base):
    # Make sure the arm is in Single Level Servoing mode
    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)

    # Move arm to ready position
    print("Moving the arm to a safe position")
    action_type = Base_pb2.RequestedActionType()
    action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
    action_list = base.ReadAllActions(action_type)
    action_handle = None
    for action in action_list.action_list:
        if action.name == "Home":
            action_handle = action.handle

    if action_handle == None:
        print("Can't reach safe position. Exiting")
        return False

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    base.ExecuteActionFromReference(action_handle)
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("Safe position reached")
    else:
        print("Timeout on action notification wait")
    return finished


def example_angular_trajectory_movement(base):
    constrained_joint_angles = Base_pb2.ConstrainedJointAngles()

    actuator_count = base.GetActuatorCount()

    # Place arm straight up
    for joint_id in range(actuator_count.count):
        joint_angle = constrained_joint_angles.joint_angles.joint_angles.add()
        joint_angle.joint_identifier = joint_id
        joint_angle.value = 0

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    print("Reaching joint angles...")
    base.PlayJointTrajectory(constrained_joint_angles)

    print("Waiting for movement to finish ...")
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("Joint angles reached")
    else:
        print("Timeout on action notification wait")
    return finished


def example_cartesian_trajectory_movement(base, base_cyclic):
    constrained_pose = Base_pb2.ConstrainedPose()

    feedback = base_cyclic.RefreshFeedback()

    cartesian_pose = constrained_pose.target_pose
    cartesian_pose.x = feedback.base.tool_pose_x  # (meters)
    cartesian_pose.y = feedback.base.tool_pose_y - 0.1  # (meters)
    cartesian_pose.z = feedback.base.tool_pose_z - 0.2  # (meters)
    cartesian_pose.theta_x = feedback.base.tool_pose_theta_x  # (degrees)
    cartesian_pose.theta_y = feedback.base.tool_pose_theta_y  # (degrees)
    cartesian_pose.theta_z = feedback.base.tool_pose_theta_z  # (degrees)

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    print("Reaching cartesian pose...")
    base.PlayCartesianTrajectory(constrained_pose)

    print("Waiting for movement to finish ...")
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("Angular movement completed")
    else:
        print("Timeout on action notification wait")
    return finished

def populateCartesianCoordinate(waypointInformation):
    waypoint = Base_pb2.CartesianWaypoint()
    waypoint.pose.x = waypointInformation[0]
    waypoint.pose.y = waypointInformation[1]
    waypoint.pose.z = waypointInformation[2]
    waypoint.blending_radius = waypointInformation[3]
    waypoint.pose.theta_x = waypointInformation[4]
    waypoint.pose.theta_y = waypointInformation[5]
    waypoint.pose.theta_z = waypointInformation[6]
    waypoint.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_BASE

    return waypoint

def example_trajectory(base, base_cyclic):
    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)
    product = base.GetProductConfiguration()
    waypointsDefinition = tuple(tuple())
    if (product.model == Base_pb2.ProductConfiguration__pb2.MODEL_ID_L53
            or product.model == Base_pb2.ProductConfiguration__pb2.MODEL_ID_L31):
        if (product.model == Base_pb2.ProductConfiguration__pb2.MODEL_ID_L31):
            kTheta_x = 90.6
            kTheta_y = -1.0
            kTheta_z = 150.0
            waypointsDefinition = ((0.439, 0.194, 0.448, 0.0, kTheta_x, kTheta_y, kTheta_z),
                                   (0.200, 0.150, 0.400, 0.0, kTheta_x, kTheta_y, kTheta_z),
                                   (0.350, 0.050, 0.300, 0.0, kTheta_x, kTheta_y, kTheta_z))
        else:
            kTheta_x = 90.0
            kTheta_y = 0.0
            kTheta_z = 90.0
            waypointsDefinition = ((0.7, 0.0, 0.5, 0.0, kTheta_x, kTheta_y, kTheta_z),
                                   (0.7, 0.0, 0.33, 0.1, kTheta_x, kTheta_y, kTheta_z),
                                   (0.7, 0.48, 0.33, 0.1, kTheta_x, kTheta_y, kTheta_z),
                                   (0.61, 0.22, 0.4, 0.1, kTheta_x, kTheta_y, kTheta_z),
                                   (0.7, 0.48, 0.33, 0.1, kTheta_x, kTheta_y, kTheta_z),
                                   (0.63, -0.22, 0.45, 0.1, kTheta_x, kTheta_y, kTheta_z),
                                   (0.65, 0.05, 0.33, 0.0, kTheta_x, kTheta_y, kTheta_z))
    else:
        print("Product is not compatible to run this example please contact support with KIN number bellow")
        print("Product KIN is : " + product.kin())

    waypoints = Base_pb2.WaypointList()

    waypoints.duration = 0.0
    waypoints.use_optimal_blending = False

    index = 0
    for waypointDefinition in waypointsDefinition:
        waypoint = waypoints.waypoints.add()
        waypoint.name = "waypoint_" + str(index)
        waypoint.cartesian_waypoint.CopyFrom(populateCartesianCoordinate(waypointDefinition))
        index = index + 1

        # Verify validity of waypoints
    result = base.ValidateWaypointList(waypoints);
    if (len(result.trajectory_error_report.trajectory_error_elements) == 0):
        e = threading.Event()
        notification_handle = base.OnNotificationActionTopic(check_for_end_or_abort(e),
                                                             Base_pb2.NotificationOptions())

        print("Moving cartesian trajectory...")

        base.ExecuteWaypointTrajectory(waypoints)

        print("Waiting for trajectory to finish ...")
        finished = e.wait(TIMEOUT_DURATION)
        base.Unsubscribe(notification_handle)

        if finished:
            print("Cartesian trajectory with no optimization completed ")
            e_opt = threading.Event()
            notification_handle_opt = base.OnNotificationActionTopic(check_for_end_or_abort(e_opt),
                                                                     Base_pb2.NotificationOptions())

            waypoints.use_optimal_blending = True
            base.ExecuteWaypointTrajectory(waypoints)

            print("Waiting for trajectory to finish ...")
            finished_opt = e_opt.wait(TIMEOUT_DURATION)
            base.Unsubscribe(notification_handle_opt)

            if (finished_opt):
                print("Cartesian trajectory with optimization completed ")
            else:
                print("Timeout on action notification wait for optimized trajectory")

            return finished_opt
        else:
            print("Timeout on action notification wait for non-optimized trajectory")

        return finished

    else:
        print("Error found in trajectory")
        result.trajectory_error_report.PrintDebugString();


def traj_gen_task(x_s, x_g, t, Tf):

    """
    path plan in Task space
    x_s = Start point cartesian
    x_g = goal point cartesian
    """

    # x_s = np.array(direct_kinematics(q1))   #start point
    # x_g = np.array(direct_kinematics(q2))   #goal point
    a0 = np.zeros((3,))
    a1 = np.zeros((3,))
    a2 = 3 / Tf ** 2
    a3 = -2 / Tf ** 3
    x = x_s + (a0 + a1 * t + a2 * t ** 2 + a3 * t ** 3) * (x_g - x_s)
    dx = (a1 + 2 * a2 * t + 3 * a3 * t ** 2) * (x_g - x_s)
    ddx = (2 * a2 + 6 * a3 * t) * (x_g - x_s)
    # q = np.array([inverse_kinematics(x[0], x[1], x[2])])
    # J = np.array(
    #     [[-l2 * np.sin(q[:, 0][0]) - l3 * np.sin(q[:, 0][0] + q[:, 1][0]), -l3 * np.sin(q[:, 0][0] + q[:, 1][0]), 0],
    #      [l2 * np.cos(q[:, 0][0]) + l3 * np.cos(q[:, 0][0] + q[:, 1][0]), l3 * np.cos(q[:, 0][0] + q[:, 1][0]), 0],
    #      [0, 0, -1]])
    #
    # J_1 = np.linalg.inv(J)
    # dq = J_1 @ np.array([[dx[0]], [dx[1]], [dx[2]]])
    #
    # dJ = np.array([[-l2 * np.cos(q[0, 0]) * dq[0, 0] - l3 * np.cos(q[0, 0] + q[0, 1]) * (dq[0, 0] + dq[1, 0]),
    #                 -l3 * np.cos(q[0, 0] + q[0, 1]) * (dq[0, 0] + dq[1, 0]), 0],
    #                [-l2 * np.sin(q[0, 0]) * dq[0, 0] - l3 * np.sin(q[0, 0] + q[0, 1]) * (dq[0, 0] + dq[1, 0]),
    #                 -l3 * np.sin(q[0, 0] + q[0, 1]) * (dq[0, 0] + dq[1, 0]), 0],
    #                [0, 0, 0]])
    #
    # ddq = J_1 @ (ddx - (dJ @ dq).reshape((3,)))

    # return q.reshape((3,)), dq.reshape((3,)), ddq.reshape((3,))

if __name__ == "__main__":

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
            base = BaseClient(router)
            base_cyclic = BaseCyclicClient(router)

            input("Remove any objects near the arm and press Enter")
            # Example core
            success = True
            flag = True
            display = True

            while flag and success:

                if display:
                    key = input("Press H to move the arm  to home position\n"
                          "Press C to move the arm to desired cartesian action\n"
                          "Press A to move the arm to desired angular action\n"
                          "To Quit press Q")
                    display = False

                if str(key) == 'h' or 'H':
                    success &= example_move_to_home_position(base)
                    if success:
                        print('Successfully moved to home position')
                        display = True
                    else:
                        print('Huston, we have a problem, please call the instructor')

                if str(key) == 'c' or 'C':
                    success &= example_cartesian_action_movement(base, base_cyclic)
                    if success:
                        print('Successfully moved to arm to desired cartesian action')
                        display = True
                    else:
                        print('Huston, we have a problem, please call the instructor')

                if str(key) == 'A' or 'a':
                    success &= example_angular_action_movement(base)
                    if success:
                        print('Successfully moved to arm to desired angular action')
                        display = True
                    else:
                        print('Huston, we have a problem, please call the instructor')

                success &= example_trajectory(base, base_cyclic)

                # if str(key) == 'T' or 't':
                #     success &= example_cartesian_trajectory_movement(base, base_cyclic)
                #     if success:
                #         print('Successfully moved to arm to desired angular action')
                #         display = True
                #     else:
                #         print('Huston, we have a problem, please call the instructor')
                #
                # if str(key) == 'Y' or 'y':
                #     success &= example_angular_trajectory_movement(base)
                #     if success:
                #         print('Successfully moved to arm to desired angular action')
                #         display = True
                #     else:
                #         print('Huston, we have a problem, please call the instructor')


        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    except KeyboardInterrupt:
        pass

