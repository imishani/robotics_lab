#! /usr/bin/env python3

import numpy as np
import sys, select, os, time
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient


e = """
Communications Failed
"""

def getKey():
    if os.name == 'nt':
        return msvcrt.getche()
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def record(base_cyclic):

    cur_joint = np.zeros(len(base_cyclic.RefreshFeedback().actuators))
    joint_list = None
    xyz_list = None

    while True:
        try:
            for i in range(len(base_cyclic.RefreshFeedback().actuators)):
                cur_joint[i] = base_cyclic.RefreshFeedback().actuators[i].position
                cur_end_xyz = np.array([base_cyclic.RefreshFeedback().base.tool_pose_x,
                               base_cyclic.RefreshFeedback().base.tool_pose_y,
                               base_cyclic.RefreshFeedback().base.tool_pose_z])
            if joint_list is None:
                joint_list = cur_joint
                xyz_list = np.array([base_cyclic.RefreshFeedback().base.tool_pose_x,
                               base_cyclic.RefreshFeedback().base.tool_pose_y,
                               base_cyclic.RefreshFeedback().base.tool_pose_z])

            else:
                joint_list = np.vstack((joint_list, cur_joint))
                xyz_list = np.vstack((xyz_list, cur_end_xyz))

            print("TCP position X {}, Y {}, Z {} \n To stop recording press Ctrl+C".format(*cur_end_xyz))
            print("Robot joints q1 {}, q2 {}, q3 {}, q4 {}, q5 {}, q6 {} \n To stop recording press Ctrl+C".format(*cur_joint))

        except KeyboardInterrupt:
            return (joint_list, xyz_list)

def save():
    global joint_trajectory
    logdir_prefix = 'lab-01'

    data_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../Lab1/data')

    if not (os.path.exists(data_path)):
        os.makedirs(data_path)

    logdir = logdir_prefix + '_' + time.strftime("%d-%m-%Y_%H-%M-%S")
    logdir = os.path.join(data_path, logdir)
    if not (os.path.exists(logdir)):
        os.makedirs(logdir)

    print("\n\n\nLOGGING TO: ", logdir, "\n\n\n")

    import pickle
    with open(logdir + '/data' + '.pkl', 'wb') as h:
        pickle.dump(joint_trajectory, h)

if __name__ == "__main__":

    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
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
            input("Connect joystick and press any key to continue")

            try:
                print("Press s to start recording")
                status = 1
                while True:
                    key = getKey()
                    if str(key) == "b's'":
                        joint_trajectory = record(base_cyclic)
                        print('Press d to save the data or q to stop and quit')
                        status = status + 1
                    elif str(key) == "b'd'":
                        save()
                        print('Press q to stop and quit')
                        status = status + 1
                    elif str(key) == "b'q'":
                        break
            except:
                print(e)
            finally:
                print('\nDone')

        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    except KeyboardInterrupt:
        pass

