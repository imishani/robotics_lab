#! /usr/bin/env python3

import numpy as np
import sys, select, os, time
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient

TIMEOUT_DURATION = 20
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


def save_data(data_to_save):
    logdir_prefix = 'lab-07'

    data_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), './data')

    if not (os.path.exists(data_path)):
        os.makedirs(data_path)

    logdir = logdir_prefix + '_' + time.strftime("%d-%m-%Y_%H-%M-%S")
    logdir = os.path.join(data_path, logdir)
    if not (os.path.exists(logdir)):
        os.makedirs(logdir)

    print("\n\n\nLOGGING TO: ", logdir, "\n\n\n")

    import pickle
    with open(logdir + '/data' + '.pkl', 'wb') as h:
        pickle.dump(data_to_save, h)

if __name__ == "__main__":

    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
    try:
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../.."))
        import utilities

        # Parse arguments
        args = utilities.parseConnectionArguments()

        # Create connection to the device and get the router
        with utilities.DeviceConnection.createTcpConnection(args) as router:
            # Create required services
            base = BaseClient(router)
            base_cyclic = BaseCyclicClient(router)

            try:
                joint_trajectory = record(base_cyclic)
                save()
            except:
                print(e)
            finally:
                print('done')

        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    except KeyboardInterrupt:
        pass

