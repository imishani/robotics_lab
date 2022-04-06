#! /usr/bin/env python3

import sys, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

import time
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2

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

            input("Move the arm above the finger and press Enter")
            exp_time = 1 * 60 * 60  # in seconds
            # Example core
            success = True
            start_time = time.time()
            success &= OpeningGripperCommands(base)

            while time.time() - start_time > exp_time:

                    success &= ClosingGripperCommands(base, value=0.5)
                    success &= OpeningGripperCommands(base)
                    print('elapsed time: ' + str(time.time() - start_time))

    except KeyboardInterrupt:
        pass

