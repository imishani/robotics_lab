#! /usr/bin/env python3

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../common/robot"))
from robot_actions import *

# Maximum allowed waiting time during actions (in seconds)
TIMEOUT_DURATION = 20
e = """
Communications Failed
"""

if __name__ == "__main__":

    try:
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
                    print('\n-----------------------------------------------')
                    key = input("Press H to move the arm  to home position\n"
                          "Press C to move the arm to desired cartesian action\n"
                          "Press A to move the arm to desired angular action\n"
                          "To Quit press Q\n")
                    display = False

                if str(key) == 'h' or str(key) == 'H':
                    success &= example_move_to_home_position(base)
                    if success:
                        print('Successfully moved to home position')
                        display = True
                    else:
                        print('Huston, we have a problem, please call the instructor')

                if str(key) == 'c' or str(key) == 'C':
                    success &= example_cartesian_action_movement(base, base_cyclic)
                    if success:
                        print('Successfully moved to arm to desired cartesian action')
                        display = True
                    else:
                        print('Huston, we have a problem, please call the instructor')

                if str(key) == 'A' or str(key) == 'a':
                    success &= example_angular_action_movement(base, base_cyclic)
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

