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
import numpy as np
from sympy import *
import matplotlib.pyplot as plt
import math

e = """
Communications Failed
"""


alpha0,alpha1,alpha2,alpha3,alpha4,alpha5,alpha6 = symbols('alpha0:7')
a1,a2,a3,a4,a5,a6 = symbols('a1:7')
d1,d2,d3,d4,d5,d6 = symbols('d1:7')
q1,q2,q3,q4,q5,q6 = symbols('q1:7')


subs_dict = {   alpha1: pi/2    , a1:0          , d1: 128.3+115.0   , q1: q1,
                alpha2:pi       , a2:280        , d2: 30            , q2: q2 + pi/2,
				alpha3:pi/2     , a3:0          , d3: 20            , q3: q3 + pi/2,
                alpha4:pi /2    , a4:0          , d4: 140.0+105.0   , q4: q4 + pi/2,
                alpha5: pi/2   , a5:0          , d5: 28.5+28.5     , q5: q5 + pi,
                alpha6:0        , a6:0          , d6: 105.0+130.0   , q6: q6 + pi/2 }

# Define Modified DH Transformation matrix

def TF_matrix(alpha,a,d,q):
        TF = Matrix([[cos(q),   -cos(alpha)*sin(q),  sin(q)*sin(alpha), a*cos(q)],
                    [sin(q),    cos(alpha)*cos(q),   -sin(alpha)*cos(q), a*sin(q)],
                    [0,         sin(alpha),         cos(alpha),          d],
                    [   0,      0,          0,                              1]])
        return TF

T_01 = TF_matrix(alpha1,a1,d1,q1).subs(subs_dict)
T_12 = TF_matrix(alpha2,a2,d2,q2).subs(subs_dict)
T_23 = TF_matrix(alpha3,a3,d3,q3).subs(subs_dict)
T_34 = TF_matrix(alpha4,a4,d4,q4).subs(subs_dict)
T_45 = TF_matrix(alpha5,a5,d5,q5).subs(subs_dict)
T_56 = TF_matrix(alpha6,a6,d6,q6).subs(subs_dict)

T = T_01*T_12*T_23*T_34*T_45*T_56

trial = np.matrix(T.evalf(subs = {q1:0.0,q2:0.0,q3:0.0,q4:0.0,q5:0.0,q6:0.0}))
#r_vec = np.matrix([[0],[0],[0],[1]])


q1_list = np.linspace(-1,1,10)
q2_list = np.linspace(-1,1,5)

x_pos =[]
y_pos =[]
output_theta = []

for i in range(len(q1_list)):
    for  j in range(len(q2_list)):
        r_vec = np.matrix(T.evalf(subs = {q1:q1_list[i],q2:q2_list[j],q3:0}))*np.matrix([[0],[0],[0],[1]])
        output_theta.append([q1_list[i],q2_list[j]])
        x_pos.append(float(r_vec[0][0]))
        y_pos.append(float(r_vec[1][0]))

plt.scatter(x_pos[:],y_pos[:])
plt.show()

class Robot:
    def __init__(self):
        if os.name != 'nt':
            self.settings = termios.tcgetattr(sys.stdin)
        # Parse arguments
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
        import utilities
        args = utilities.parseConnectionArguments()
        # Create connection to the device and get the router
        with utilities.DeviceConnection.createTcpConnection(args) as router:
            # Create required services
            self.base = BaseClient(router)
            self.base_cyclic = BaseCyclicClient(router)



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

            print("Curr Gripper X {}, Y {}, Z {} \n To stop recording press Ctrl+C".format(*cur_end_xyz))

        except KeyboardInterrupt:
            return (joint_list, xyz_list)

def save():
    global joint_trajectory
    logdir_prefix = 'lab-01'

    data_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../data')

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

    robot = Robot()

    try:
        print("Press s to start recording")
        status = 1
        while (1):
            key = getKey()
            if str(key) == "b's'":
                joint_trajectory = record(robot.base_cyclic)
                print('Press d to save the data')
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
        print('done')




