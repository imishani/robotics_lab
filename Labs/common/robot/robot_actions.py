#! /usr/bin/env python3

import numpy as np
import sys, select, os, time, threading ,signal
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2

import utilities
# Maximum allowed waiting time during actions (in seconds)
TIMEOUT_DURATION = 1000
e = """
Communications Failed
"""


if os.name != 'nt':
    settings = termios.tcgetattr(sys.stdin)


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


def move_to_press_position(base):
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
        if action.name == "press":
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


def example_angular_action_movement(base, base_cyclic, Q=None):
    print("Starting angular action movement ...")
    action = Base_pb2.Action()
    action.name = "Example angular action movement"
    action.application_data = ""
    ok = False
    actuator_count = base.GetActuatorCount().count
    if Q is not None:
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


def example_cartesian_action_movement(base, base_cyclic, C=None):
    print("Starting Cartesian action movement ...")

    action = Base_pb2.Action()
    action.name = "Example Cartesian action movement"
    action.application_data = ""

    feedback = base_cyclic.RefreshFeedback()
    if np.array(C).any():
        cartesian_pose = action.reach_pose.target_pose
        cartesian_pose.x = C[0][0]  # (meters)
        cartesian_pose.y = C[1][0]  # (meters)
        cartesian_pose.z = C[2][0]  # (meters)
        cartesian_pose.theta_x = C[3][0]  # (degrees)
        cartesian_pose.theta_y = C[4][0]  # (degrees)
        cartesian_pose.theta_z = C[5][0]  # (degrees)
    else:
        ok = False
        while not ok:
            d_y = float(input("Enter required delta y for the end effector position: "))
            d_x = float(input("Enter required delta x for the end effector position: "))
            if abs(d_y) < 0.2 and abs(d_x) < 0.2:
                print('Input deltas are ok, executing')
                ok = True
            else:
                print('Only less then 0.2 [m] delta inputs are valid.')

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
        print("Cartesian movement completed \nCheck EVENT if the arm reached your destination")
    else:
        print("Timeout\Abort on action notification wait")
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


def ClosingGripperCommands(base, value=0.5):

    # Create the GripperCommand we will send
    gripper_command = Base_pb2.GripperCommand()
    finger = gripper_command.gripper.finger.add()

    # Close the gripper with position increments
    print("Performing gripper test in position...")
    gripper_command.mode = Base_pb2.GRIPPER_POSITION
    position = 0.00
    finger.finger_identifier = 1
    while position < value:
        finger.value = position
        print("Going to position {:0.2f}...".format(finger.value))
        base.SendGripperCommand(gripper_command)
        position += 0.1
        time.sleep(0.2)
    return True


def OpeningGripperCommands(base):

    # Create the GripperCommand we will send
    gripper_command = Base_pb2.GripperCommand()
    finger = gripper_command.gripper.finger.add()

    # Set speed to open gripper
    print ("Opening gripper using speed command...")
    gripper_command.mode = Base_pb2.GRIPPER_SPEED
    finger.value = 0.8
    base.SendGripperCommand(gripper_command)
    gripper_request = Base_pb2.GripperRequest()

    # Wait for reported position to be opened
    gripper_request.mode = Base_pb2.GRIPPER_POSITION
    while True:
        gripper_measure = base.GetMeasuredGripperMovement(gripper_request)
        if len (gripper_measure.finger):
            print("Current position is : {0}".format(gripper_measure.finger[0].value))
            if gripper_measure.finger[0].value < 0.122:
                break
        else: # Else, no finger present in answer, end loop
            break
    return True

def move_to_angle_conf(angle_conf_eval):
    args = utilities.parseConnectionArguments()
    with utilities.DeviceConnection.createTcpConnection(args) as router:
        # Create required services
        base = BaseClient(router)
        base_cyclic = BaseCyclicClient(router)
        input("Remove any objects near the arm and press Enter")
        for i in range(len(angle_conf_eval)):

            Q = angle_conf_eval['target']
            # Create connection to the device and get the router
            # Example core
            success = True
            flag = True
            display = True

            while flag and success:

                if display:
                    key = input("Press H to move the arm  to home position\n"
                                "Press A to move the arm to desired angular position: \n"
                                + str(np.round(Q.squeeze(), 3)) + '\n'
                                + "To Quit press Q\n")
                    display = False

                if str(key) == 'h' or str(key) == 'H':
                    success &= example_move_to_home_position(base)
                    if success:
                        print('Successfully moved to home position')
                        display = True
                    else:
                        print('Huston, we have a problem, please call the instructor')

                if str(key) == 'A' or str(key) == 'a':
                    success &= example_angular_action_movement(base, base_cyclic, Q=Q)
                    if success:
                        print('Successfully moved to arm to desired angular action')
                        flag = False
                    else:
                        print('Huston, we have a problem, please call the instructor')
                if str(key) == 'q' or str(key) == 'Q':
                    break


def move_to_multiple_angle_conf(angle_conf_eval):

    args = utilities.parseConnectionArguments()
    with utilities.DeviceConnection.createTcpConnection(args) as router:
        # Create required services
        base = BaseClient(router)
        base_cyclic = BaseCyclicClient(router)
        input("Remove any objects near the arm and press Enter")
        input("Moving the robot to home position")
        example_move_to_home_position(base)
        print()

        quit_flag = False
        for i in range(len(angle_conf_eval)):

            C = angle_conf_eval['t' + str(i + 1)]

            success = True
            flag = True
            display = True

            while flag and success and not quit_flag:

                if display:
                    key = input("Press H to move the arm  to home position\n"
                                "Press A to move the arm to desired cartesian position: \n"
                                + str(np.round(C.squeeze(), 3)) + '\n'
                                + "To Quit press Q\n")
                    display = False

                if str(key) == 'h' or str(key) == 'H':
                    success &= example_move_to_home_position(base)
                    if success:
                        print('Successfully moved to home position')
                        display = True
                    else:
                        print('Huston, we have a problem, please call the instructor')

                if str(key) == 'A' or str(key) == 'a':
                    success &= example_cartesian_action_movement(base, base_cyclic, C=C)
                    if success:
                        print('Successfully moved to arm to desired cartesian action\n\n')
                        flag = False
                    else:
                        print('Huston, we have a problem, please call the instructor')
                if str(key) == 'q' or str(key) == 'Q':
                    quit_flag = True
                    break


def trajectory_task(base, base_cyclic, traj_gen_task, goals, Tf=3., N=5):
    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)
    product = base.GetProductConfiguration()
    waypointsDefinition = goals  # tuple(tuple())


    point_num = 1
    for x_g in waypointsDefinition:  # waypointsDefinition
        x_s = np.array([base_cyclic.RefreshFeedback().base.tool_pose_x,
                        base_cyclic.RefreshFeedback().base.tool_pose_y,
                        base_cyclic.RefreshFeedback().base.tool_pose_z,
                        0.,
                        base_cyclic.RefreshFeedback().base.tool_pose_theta_x,
                        base_cyclic.RefreshFeedback().base.tool_pose_theta_y,
                        base_cyclic.RefreshFeedback().base.tool_pose_theta_z])

        t = np.linspace(0, Tf, N)
        waypointsDef = [tuple(traj_gen_task(x_s, np.array(list(x_g)), ti, Tf)[0]) for ti in t]
        waypoints = Base_pb2.WaypointList()
        waypoints.duration = 0.0
        waypoints.use_optimal_blending = True

        from tqdm.auto import tqdm
        prog_bar = tqdm(waypointsDef)
        index = 0
        for waypointDef in prog_bar:
            waypoint = waypoints.waypoints.add()
            waypoint.name = "waypoint_" + str(index)
            waypoint.cartesian_waypoint.CopyFrom(populateCartesianCoordinate(waypointDef))
            index = index + 1
            prog_bar.set_description(f'Added point {index} to trajectory of x goal {x_g}')
            prog_bar.refresh()

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

                print("Cartesian trajectory with optimization completed, Path num: " + str(point_num))
                point_num += 1
                if point_num == 4:
                    ClosingGripperCommands(base, 0.7)

                else:
                    print("Timeout on action notification wait for optimized trajectory")
                # finished_ = finished_opt

            else:
                print("Timeout on action notification wait for non-optimized trajectory")

        else:
            print("Error found in trajectory")
            result.trajectory_error_report.PrintDebugString();

    return finished


def trajectory_config(base, base_cyclic, traj_gen_config, angles, Tf=3., N=3):
    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)
    product = base.GetProductConfiguration()
    jointPoses = angles  # tuple(tuple())

    point_num = 1
    for q_g in jointPoses:
        q_g = np.array(q_g)
        q_s = np.zeros(len(base_cyclic.RefreshFeedback().actuators))
        for i in range(len(base_cyclic.RefreshFeedback().actuators)):
            q_s[i] = base_cyclic.RefreshFeedback().actuators[i].position

        t = np.linspace(0, Tf, N)

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

            base.ExecuteWaypointTrajectory(waypoints)

            print("Waiting for trajectory to finish ...")
            finished = e.wait(100)
            base.Unsubscribe(notification_handle)

            if finished:
                print("Angular movement completed,   Path num: " + str(point_num))
                point_num += 1
                if point_num == 4:
                    ClosingGripperCommands(base, 0.7)
            else:
                print("Timeout on action notification wait")
        else:
            print("Error found in trajectory")
            print(result.trajectory_error_report)
            # return finished
    return finished