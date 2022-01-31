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
import time
import threading
import signal
import subprocess
import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2


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

def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    return True

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

def move_to_home_fixed(base):
    Q = [10.0, 340.0, 75.0, 340.0, 300.0, 10.0]
    action = Base_pb2.Action()
    action.name = "Example angular action movement"
    action.application_data = ""
    actuator_count = base.GetActuatorCount()

    for joint_id in range(actuator_count.count):
        joint_angle = action.reach_joint_angles.joint_angles.joint_angles.add()
        joint_angle.value = Q[joint_id]

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

def move_to_home_position(base):
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

def populateAngularPose(jointPose, durationFactor):
    waypoint = Base_pb2.AngularWaypoint()
    waypoint.angles.extend(jointPose)
    waypoint.duration = durationFactor * 5.0

    return waypoint

def trajectory_task(base, points):

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

    x_g = waypointsDefinition[-1]
    x_s = (base_cyclic.RefreshFeedback().base.tool_pose_x,
           base_cyclic.RefreshFeedback().base.tool_pose_y,
           base_cyclic.RefreshFeedback().base.tool_pose_z,
           0.,
           base_cyclic.RefreshFeedback().base.tool_pose_theta_x,
           base_cyclic.RefreshFeedback().base.tool_pose_theta_y,
           base_cyclic.RefreshFeedback().base.tool_pose_theta_z)

    Tf = 3.
    t = np.linspace(0, Tf, 5)
    waypointsDef = [tuple(traj_gen_task(x_s, x_g, ti, Tf)[0]) for ti in t]
    waypoints = Base_pb2.WaypointList()

    waypoints.duration = 0.0
    waypoints.use_optimal_blending = False

    index = 0
    # for waypointDefinition in waypointsDefinition:
    #     waypoint = waypoints.waypoints.add()
    #     waypoint.name = "waypoint_" + str(index)
    #     waypoint.cartesian_waypoint.CopyFrom(populateCartesianCoordinate(waypointDefinition))
    #     index = index + 1
    for waypointDef in waypointsDef:
        waypoint = waypoints.waypoints.add()
        waypoint.name = "waypoint_" + str(index)
        waypoint.cartesian_waypoint.CopyFrom(populateCartesianCoordinate(waypointDef))
        index = index + 1

        # Verify validity of waypoints
    result = base.ValidateWaypointList(waypoints);
    if (len(result.trajectory_error_report.trajectory_error_elements) == 0):
        e = threading.Event()
        notification_handle = base.OnNotificationActionTopic(check_for_end_or_abort(e),
                                                             Base_pb2.NotificationOptions())

        print("Moving cartesian trajectory...")
        cmd = 'start cmd /D /C "python recorder.py && pause"'
        pro = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                               shell=True, stdin=subprocess.PIPE)
        time.sleep(1)


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

def trajectory_config(base, angles):
    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)

    jointPoses = tuple(tuple())
    product = base.GetProductConfiguration()

    if (product.model == Base_pb2.ProductConfiguration__pb2.MODEL_ID_L53
            or product.model == Base_pb2.ProductConfiguration__pb2.MODEL_ID_L31):
        if (product.model == Base_pb2.ProductConfiguration__pb2.MODEL_ID_L31):
            jointPoses = ((0.0, 344.0, 75.0, 360.0, 300.0, 0.0),
                          (0.0, 21.0, 145.0, 272.0, 32.0, 273.0),
                          (42.0, 334.0, 79.0, 241.0, 305.0, 56.0))
        else:
            # Binded to degrees of movement and each degrees correspond to one degree of liberty
            degreesOfFreedom = base.GetActuatorCount();

            if (degreesOfFreedom.count == 6):
                jointPoses = ((360.0, 35.6, 281.8, 0.8, 23.8, 88.9),
                              (359.6, 49.1, 272.1, 0.3, 47.0, 89.1),
                              (320.5, 76.5, 335.5, 293.4, 46.1, 165.6),
                              (335.6, 38.8, 266.1, 323.9, 49.7, 117.3),
                              (320.4, 76.5, 335.5, 293.4, 46.1, 165.6),
                              (28.8, 36.7, 273.2, 40.8, 39.5, 59.8),
                              (360.0, 45.6, 251.9, 352.2, 54.3, 101.0))
            else:
                jointPoses = ((360.0, 35.6, 180.7, 281.8, 0.8, 23.8, 88.9),
                              (359.6, 49.1, 181.0, 272.1, 0.3, 47.0, 89.1),
                              (320.5, 76.5, 166.5, 335.5, 293.4, 46.1, 165.6),
                              (335.6, 38.8, 177.0, 266.1, 323.9, 49.7, 117.3),
                              (320.4, 76.5, 166.5, 335.5, 293.4, 46.1, 165.6),
                              (28.8, 36.7, 174.7, 273.2, 40.8, 39.5, 59.8),
                              (360.0, 45.6, 171.0, 251.9, 352.2, 54.3, 101.0))

    else:
        print("Product is not compatible to run this example please contact support with KIN number bellow")
        print("Product KIN is : " + product.kin())

    q_g = np.array(jointPoses[-1])
    q_s = np.zeros(len(base_cyclic.RefreshFeedback().actuators))
    for i in range(len(base_cyclic.RefreshFeedback().actuators)):
        q_s[i] = base_cyclic.RefreshFeedback().actuators[i].position

    Tf = 3.
    t = np.linspace(0, Tf, 3)
    jointP = [tuple(traj_gen_config(q_s, q_g, ti, Tf)[0]) for ti in t]

    waypoints = Base_pb2.WaypointList()
    waypoints.duration = 0.0
    waypoints.use_optimal_blending = False

    index = 0

    for jointPose in jointP:
        waypoint = waypoints.waypoints.add()
        waypoint.name = "waypoint_" + str(index)
        durationFactor = 1
        # Joints/motors 5 and 7 are slower and need more time
        if (index == 4 or index == 6):
            durationFactor = 6  # Min 30 seconds

        waypoint.angular_waypoint.CopyFrom(populateAngularPose(jointPose, durationFactor))
        index = index + 1

        # Verify validity of waypoints
    result = base.ValidateWaypointList(waypoints);
    if (len(result.trajectory_error_report.trajectory_error_elements) == 0):

        e = threading.Event()
        notification_handle = base.OnNotificationActionTopic(
            check_for_end_or_abort(e),
            Base_pb2.NotificationOptions()
        )

        print("Reaching angular pose trajectory...")
        cmd = 'start cmd /D /C "python recorder.py && pause"'
        pro = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                               shell=True, stdin=subprocess.PIPE)
        time.sleep(1.)

        base.ExecuteWaypointTrajectory(waypoints)

        print("Waiting for trajectory to finish ...")
        finished = e.wait(TIMEOUT_DURATION)
        base.Unsubscribe(notification_handle)

        if finished:
            print("Angular movement completed")
        else:
            print("Timeout on action notification wait")
        return finished
    else:
        print("Error found in trajectory")
        print(result.trajectory_error_report)
        return finished

def traj_gen_config(q1, q2, t, Tf):
    ''' path plan configuration space '''
    qm = q1 + (q2 - q1)/2
    a0 = q1
    a1 = np.zeros((6,))
    a4 = (qm - 0.5 * (q1 + q2)) / ((Tf / 2) ** 4)
    a3 = (2 * (q1 - q2) / (Tf ** 3)) - 2 * Tf * a4
    a2 = -1.5 * a3 * Tf - 2 * a4 * Tf ** 2

    q = a0 + a1 * t + a2 * t ** 2 + a3 * t ** 3 + a4 * t ** 4
    dq = a1 + 2 * a2 * t + 3 * a3 * t ** 2 + 4 * a4 * t ** 3
    ddq = 2 * a2 + 6 * a3 * t + 12 * a4 * t ** 2

    return q, dq, ddq

def traj_gen_task(x_s, x_g, t, Tf):

    """
    path plan in Task space
    x_s = Start point cartesian
    x_g = goal point cartesian
    """
    x_s = np.array(list(x_s))   #start point
    x_g = np.array(list(x_g))   #goal point
    a0 = 0. # np.zeros((3,))
    a1 = 0. # np.zeros((3,))
    a2 = 3 / Tf ** 2
    a3 = -2 / Tf ** 3
    x = x_s + (a0 + a1 * t + a2 * t ** 2 + a3 * t ** 3) * (x_g - x_s)
    dx = (a1 + 2 * a2 * t + 3 * a3 * t ** 2) * (x_g - x_s)
    ddx = (2 * a2 + 6 * a3 * t) * (x_g - x_s)

    return x, dx, ddx


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
                          "Press C to follow a trajectory on task-space\n"
                          "Press A to follow a trajectory on configuration space\n"
                          "To Quit press Q")
                    print(str(key))
                    display = False

                if success:
                    print('Successfully moved')
                    display = True
                else:
                    print('Huston, we have a problem, please call the instructor')

                if str(key) == 'h' or str(key) == 'H':
                    success &= move_to_home_position(base)
                    if success:
                        print('Successfully moved to home position')
                        display = True
                    else:
                        print('Huston, we have a problem, please call the instructor')

                if str(key) == 'c' or str(key) == 'C':
                    waypoints = []
                    success &= trajectory_task(base, waypoints)
                    if success:
                        print('Successfully moved')
                        display = True
                    else:
                        print('Huston, we have a problem, please call the instructor')

                if str(key) == 'a' or str(key) == 'A':
                    waypoints = []
                    success &= move_to_home_fixed(base)
                    success &= trajectory_config(base, waypoints)
                    if success:
                        print('Successfully moved to arm to desired angular action')
                        display = True
                    else:
                        print('Huston, we have a problem, please call the instructor')




        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    except KeyboardInterrupt:
        pass

