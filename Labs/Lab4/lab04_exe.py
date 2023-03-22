#! /usr/bin/env python3

import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../common/robot"))
from robot_actions import *
from lab04_solution import *


if __name__ == "__main__":

    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
    try:

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

                    import subprocess
                    path = os.path.join(os.path.dirname(__file__), "../common/robot/recorder.py")
                    cmd = f'start cmd /D /C "python {path} && pause"'
                    pro = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                           shell=True, stdin=subprocess.PIPE)
                    time.sleep(1)

                    success &= trajectory_task(base, base_cyclic, traj_gen_task, waypoints)
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

                    import subprocess
                    path = os.path.join(os.path.dirname(__file__), "../common/robot/recorder.py")
                    cmd = f'start cmd /D /C "python {path} && pause"'
                    pro = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                           shell=True, stdin=subprocess.PIPE)
                    time.sleep(1)

                    success &= trajectory_config(base, base_cyclic, traj_gen_config, waypoints)
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

