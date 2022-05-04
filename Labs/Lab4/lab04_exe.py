#! /usr/bin/env python3

import numpy as np
import time
import threading
import signal
import subprocess
import sys, select, os
from tqdm.auto import tqdm
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../common/robot"))
from robot_actions import *
from lab04_sol import *


def trajectory_task(base, goals, Tf=3., N=5):

    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)
    product = base.GetProductConfiguration()
    waypointsDefinition = goals     # tuple(tuple())

    cmd = 'start cmd /D /C "python recorder.py && pause"'
    pro = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                           shell=True, stdin=subprocess.PIPE)
    time.sleep(1)
    point_num = 1
    for x_g in waypointsDefinition:    # waypointsDefinition
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

                print("Cartesian trajectory with optimization completed, Path num: " + str(point_num) )
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


def trajectory_config(base, angles, Tf=3., N=3):
    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)
    product = base.GetProductConfiguration()
    jointPoses = angles       # tuple(tuple())

    cmd = 'start cmd /D /C "python recorder.py && pause"'
    pro = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                           shell=True, stdin=subprocess.PIPE)
    time.sleep(1.)
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
                    print('\n===================================================\n')
                    key = input("Press H to move the arm  to home position\n"
                                "Press G to close the gripper\n"
                                "Press O to open the gripper\n"
                                "Press C to follow a trajectory on task-space\n"
                                "Press A to follow a trajectory on configuration space\n"
                                "To Quit press Q\n")
                    print(str(key))
                    display = False

                if success:
                    display = True
                else:
                    print('Huston, we have a problem, please call the instructor')

                # Home
                if str(key) == 'h' or str(key) == 'H':
                    success &= example_move_to_home_position(base)
                    if success:
                        print('Successfully moved to home position')
                        display = True
                    else:
                        print('Huston, we have a problem, please call the instructor')
                # Close gripper
                if str(key) == 'g' or str(key) == 'G':
                    value = input("Enter the amount to close (1 is maximum 0 is minimum): ")
                    success &= ClosingGripperCommands(base, float(value))
                    if success:
                        print('Successfully closed the gripper')
                        display = True
                    else:
                        print('Huston, we have a problem, please call the instructor')
                # Open gripper
                if str(key) == 'o' or str(key) == 'O':
                    success &= OpeningGripperCommands(base)
                    if success:
                        print('Successfully opened the gripper')
                        display = True
                    else:
                        print('Huston, we have a problem, please call the instructor')
                # Trajectory task-space
                if str(key) == 'c' or str(key) == 'C':
                    waypoints = generate_x_goals_list()
                    waypoints = np.hstack((waypoints[:, :3], np.zeros((waypoints.shape[0], 1)), waypoints[:, 3:]))
                    waypoints = tuple(tuple(row) for row in waypoints)
                    OpeningGripperCommands(base)

                    success &= trajectory_task(base, waypoints)
                    if success:
                        print('Successfully moved')
                        display = True
                    else:
                        print('Huston, we have a problem, please call the instructor')
                # Trajectory conf-space
                if str(key) == 'a' or str(key) == 'A':
                    waypoints = generate_q_goals_list()
                    waypoints = tuple(tuple(row) for row in waypoints)

                    success &= move_to_home_fixed(base)
                    OpeningGripperCommands(base)
                    success &= trajectory_config(base, waypoints)
                    if success:
                        print('Successfully moved to arm to desired angular action')
                        display = True
                    else:
                        print('Huston, we have a problem, please call the instructor')
                if str(key) == 'q' or str(key) == 'Q':
                    break



        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    except KeyboardInterrupt:
        pass

