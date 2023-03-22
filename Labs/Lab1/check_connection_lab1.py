#! /usr/bin/env python3

import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../Lab1/"))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../common/robot"))
from robot_actions import *
import utilities

'''
This code checks the connection between a computer and a robot to ensure that they can communicate with each other.
If you experiencing any problem, please call the instructor.
'''

def main():
    # Import the utilities helper module
    try:
        # Parse arguments
        args = utilities.parseConnectionArguments()

        # Create connection to the device and get the router
        with utilities.DeviceConnection.createTcpConnection(args) as router:
            # Create required services
            base = BaseClient(router)
            base_cyclic = BaseCyclicClient(router)
            print('Connection Successful')
    except:
        print('Connection Failed')

if __name__ == "__main__":
    exit(main())