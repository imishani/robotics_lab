# Kinova Gen3_lite Inverse Kinematics Equation
# INIT Robots Lab, ETS University, Montreal
# August 2020
#****************** Joint position limits **********************#
# joint 1: Lower limit: -2.6896 *********** Upperlimit: +2.6896 #
# joint 2: Lower limit: -2.6197 *********** Upperlimit: +2.6197 #
# joint 3: Lower limit: -2.6197 *********** Upperlimit: +2.6197 #
# joint 4: Lower limit: -2.6002 *********** Upperlimit: +2.6002 #
# joint 5: Lower limit: -2.5302 *********** Upperlimit: +2.5307 #
# joint 6: Lower limit: -2.6002 *********** Upperlimit: +2.6002 #
#***************************************************************#
import math
import sys
# import rospy
import time
import numpy as np
from time import sleep
from numpy import linalg as LA
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import matplotlib.lines as mlines
from matplotlib.patches import Ellipse
from matplotlib.text import OffsetFrom
# from sensor_msgs.msg import JointState
#from function_kinova_joint_simulation import SimJointMove
#******************input_desired_position_of_end_effector**********************#
#exm = np.array([0.057, -0.01, 1.0033, 0, 0, 0])                   #Example_1
#exm = np.array([0.76, -0.01, 0.1863, -1.3, 1.5, -1.3])            #Example_2
#exm = np.array([0.01, 0.76, 0.1863, -1.37, 1.5, 0.197])           #Example_3
#exm = np.array([-0.01, -0.76, 0.1863, -1.37, 1.5, -2.94])         #Example_4
#exm = np.array([-0.602, 0.46, 0.1863, -1.37, 1.5, 1.12])          #Example_5
#exm = np.array([-0.614, -0.44, 0.1863, -1.37, 1.5, 2.40])         #Example_6
#exm = np.array([-0.645, 0.116, 0.267, -2.11, -0.246, 1.20])       #Example_7
#exm = np.array([-0.3849, -0.5090, 0.4861, 0.73, 0.040, -1.364])   #Example_8
#exm = np.array([0.3045, 0.041, 0.685, 1.166, -0.398, 1.735])      #Example_9
exm = np.array([0.119, -0.04, 0.763, -0.527, 0.47, -0.759])       #Example_10
#exm = np.array([-0.047, -0.596, 0.248, -1.40, 0.77, 3.02])        #Example_11
#exm = np.array([0.347, -0.509, 0.473, 1.499, 0.07, 0.565])        #Example_12
#exm = np.array([0.354, -0.046, 0.761, 0.358, 0.830, -0.29])       #Example_13
#exm = np.array([-0.379, -0.48, 0.431, -2.09, -0.065, 2.54])       #Example_14

X    = exm[0]  #input('please input desired cartesian pose,X:')
Y    = exm[1]  #input('please input desired cartesian pose,Y:')
Z    = exm[2]  #input('please input desired cartesian pose,Z:')
Roll = exm[3]  #input('please input desired rotation pose value,Roll:')
Pitch= exm[4]  #input('please input desired rotation pose value,Pitch:')
Yaw  = exm[5]  #input('please input desired rotation pose value,Yaw:')
#***************defenition_of_kinova_gen3_parameters***************************#
a2 = 0.28;  a6 = 0;       b1 = 0.2433;  b2 = 0.03;
b3 = 0.02;  b4 = 0.245;   b5 = 0.057;  b6 = 0.235;
al = np.array([0.5*math.pi, math.pi, 0.5*math.pi, 0.5*math.pi, 0.5*math.pi, 0])
q1_init =  0    #input('please input original value of joint_1,q1:')
q2_init =  0    #input('please input original value of joint_1,q1:')
q3_init =  0    #input('please input original value of joint_1,q1:')
q4_init =  0    #input('please input original value of joint_1,q1:')
q5_init =  0    #input('please input original value of joint_1,q1:')
q6_init =  0    #input('please input original value of joint_1,q1:')
#******************************************************************************#
q11  =   math.cos(Pitch)*math.cos(Yaw);
q12  =   math.cos(Yaw)*math.sin(Pitch)*math.sin(Roll) - math.cos(Roll)*math.sin(Yaw);
q13  =   math.sin(Roll)*math.sin(Yaw) + math.cos(Roll)*math.sin(Pitch)*math.cos(Yaw);
q21  =   math.cos(Pitch)*math.sin(Yaw);
q22  =   math.cos(Roll)*math.cos(Yaw) + math.sin(Yaw)*math.sin(Pitch)*math.sin(Roll);
q23  =  -math.sin(Roll)*math.cos(Yaw) + math.sin(Yaw)*math.sin(Pitch)*math.cos(Roll);
q31  =  -math.sin(Pitch);
q32  =   math.cos(Pitch)*math.sin(Roll);
q33  =   math.cos(Pitch)*math.cos(Roll);
#******************************************************************************#
P = np.array([[X],
              [Y],
              [Z]])
Q = np.array([[q11, q12, q13],
              [q21, q22, q23],
              [q31, q32, q33]])
Q6_tmA_6 = np.array([[0],
                     [0],
                    [0.235]])
R = P - np.matmul(Q, Q6_tmA_6)
r1 = R[0][0]
r2 = R[1][0]
r3 = R[2][0]
# **********************************theta_1*************************************
rt = np.roots(cX)
theta1 = np.array([])
for rx in rt:
	if rx.imag == 0:
		theta1 = np.append(theta1,2*math.atan(rx.real))
q1 = theta1 - 0
# **************wrapToPi q1****************#
xwrap=np.remainder(q1, 2*np.pi)
mask = np.abs(xwrap)>np.pi
xwrap[mask] -= 2*np.pi * np.sign(xwrap[mask])
q1 = xwrap
# ***********possible_solution_for_q1_considering_joint_limitation*************#
pos_sol_q1 = [index for index,value in enumerate(q1) if (value >= -2.68) & (value <= 2.68)]
a_numpy_array = np.array(theta1)
accessed_array = a_numpy_array[pos_sol_q1]
q1 = np.array(accessed_array)
theta1 = np.array(accessed_array)
#****************************calculate_theta_4*********************************#
th4 = np.array([])
for i in range(len(theta1)):
    th1 = theta1[i]
    s4 = ((10556001*q33**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)**3)/1000000000000 - (13262382261*q33**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)**2)/50000000000000 - (125464296693879*q33**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/10000000000000000 - (3249*q33**2*r2**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)**2)/500000 - (3249*q33**2*r3**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)**2)/500000 + (16662634196121*q13**2*math.cos(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/10000000000000000 + (16662634196121*q23**2*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/10000000000000000 + (19276685763*q33**2*r3*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/250000000000 - (1579188121*q23**2*math.cos(th1)**2*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/361000000 - (1579188121*q13**2*math.sin(th1)**2*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/361000000 + (25202457261*q13**2*math.cos(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)**2)/50000000000000 + (10556001*q13**2*math.cos(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)**3)/1000000000000 + (25202457261*q23**2*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)**2)/50000000000000 + (10556001*q23**2*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)**3)/1000000000000 + (4081989*q33**2*r2**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/50000000 + (7904817*q33**2*r3*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)**2)/2500000000 + (3915967*q33**2*r3**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/50000000 + q33**2*r2**4*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1) - (2433*q33**2*r3**3*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/2500 + q33**2*r3**4*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1) + (1947211*q13**2*r3*math.sin(th1)**2*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/54150 + (4081989*q13**2*r2**2*math.cos(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/50000000 - (7904817*q13**2*r3*math.cos(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)**2)/2500000000 - (7923011*q13**2*r1**2*math.cos(th1)**4*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/50000000 + (15920967*q13**2*r3**2*math.cos(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/50000000 - (4081989*q13**2*r2**2*math.cos(th1)**4*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/50000000 + q13**2*r2**4*math.cos(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1) - (2433*q13**2*r3**3*math.cos(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/2500 + q13**2*r3**4*math.cos(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1) - 2*q13**2*r2**4*math.cos(th1)**4*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1) + q13**2*r1**4*math.cos(th1)**6*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1) + q13**2*r2**4*math.cos(th1)**6*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1) + (4081989*q33**2*r1**2*math.cos(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/50000000 - (4081989*q33**2*r2**2*math.cos(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/50000000 - 2*q33**2*r2**4*math.cos(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1) + q33**2*r1**4*math.cos(th1)**4*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1) + q33**2*r2**4*math.cos(th1)**4*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1) + (4081989*q23**2*r2**2*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/50000000 - (7904817*q23**2*r3*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)**2)/2500000000 + (15920967*q23**2*r3**2*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/50000000 - (2401*q23**2*r2**2*math.sin(th1)**4*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/10000 + q23**2*r2**4*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1) - (2433*q23**2*r3**3*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/2500 + q23**2*r3**4*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1) - (2433*q33**2*r2**2*r3*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/2500 - (240100*q23**2*r1**2*math.cos(th1)**4*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - (240100*q23**2*r3**2*math.cos(th1)**2*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 + (5919489*q23**2*math.cos(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/25000000 - (240100*q13**2*r3**2*math.sin(th1)**2*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - (240100*q13**2*r2**2*math.sin(th1)**4*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 + (5919489*q13**2*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/25000000 - (3249*q13**2*r2**2*math.cos(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)**2)/500000 - (3249*q13**2*r1**2*math.cos(th1)**4*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)**2)/500000 + (3249*q13**2*r3**2*math.cos(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)**2)/500000 + (3249*q13**2*r2**2*math.cos(th1)**4*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)**2)/500000 + (3249*q33**2*r1**2*math.cos(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)**2)/500000 + (3249*q33**2*r2**2*math.cos(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)**2)/500000 - (3249*q23**2*r2**2*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)**2)/500000 + (3249*q23**2*r3**2*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)**2)/500000 + (3249*q33**2*r2**2*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)**2)/250000 + 2*q33**2*r2**2*r3**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1) - (9931479237*q13**2*r3*math.cos(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/250000000000 - (9931479237*q23**2*r3*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/250000000000 + (1947211*q23**2*r3*math.cos(th1)**2*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/54150 + (7904817*q23*q33*math.cos(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)**2*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/2500000000 + (1579188121*q13*q23*math.cos(th1)*math.sin(th1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/180500000 - (7904817*q13*q33*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)**2*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/2500000000 + (7904817*q13*q33*r1*math.cos(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)**2)/1250000000 + 4*q23**2*r1**2*math.cos(th1)**4*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2 + 4*q23**2*r3**2*math.cos(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2 + (7904817*q23*q33*r2*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)**2)/1250000000 + (25202457261*q13*q23*math.cos(th1)*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)**2)/25000000000000 + (10556001*q13*q23*math.cos(th1)*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)**3)/500000000000 + 4*q13**2*r3**2*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2 + 4*q13**2*r2**2*math.sin(th1)**4*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2 - (2401*q13**2*r2**2*math.cos(th1)**2*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/10000 - (7923011*q23**2*r1**2*math.cos(th1)**2*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/50000000 - (4081989*q23**2*r2**2*math.cos(th1)**2*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/50000000 - 2*q23**2*r2**4*math.cos(th1)**2*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1) + q23**2*r1**4*math.cos(th1)**4*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1) + q23**2*r2**4*math.cos(th1)**4*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1) - (2433*q13**2*r2**2*r3*math.cos(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/2500 - (2433*q13**2*r1**2*r3*math.cos(th1)**4*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/2500 + (2433*q13**2*r2**2*r3*math.cos(th1)**4*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/2500 - (2433*q33**2*r1**2*r3*math.cos(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/2500 + (2433*q33**2*r2**2*r3*math.cos(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/2500 - (2433*q23**2*r2**2*r3*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/2500 - (240100*q13**2*r1**2*math.cos(th1)**2*math.sin(th1)**2*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - (240100*q23**2*r2**2*math.cos(th1)**2*math.sin(th1)**2*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - (9931479237*q23*q33*math.cos(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/250000000000 - (3249*q23**2*r1**2*math.cos(th1)**2*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)**2)/500000 + (3249*q23**2*r2**2*math.cos(th1)**2*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)**2)/500000 + (9931479237*q13*q33*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/250000000000 + (5841633*q13*q33*r1*math.cos(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/50000000 - (2433*q23**2*r3*math.cos(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/1250 + (5841633*q23*q33*r2*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/50000000 + (16662634196121*q13*q23*math.cos(th1)*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/5000000000000000 - (2433*q13**2*r3*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/1250 + 2*q13**2*r1**2*r2**2*math.cos(th1)**4*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1) + 2*q13**2*r2**2*r3**2*math.cos(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1) + 2*q13**2*r1**2*r3**2*math.cos(th1)**4*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1) - 2*q13**2*r1**2*r2**2*math.cos(th1)**6*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1) - 2*q13**2*r2**2*r3**2*math.cos(th1)**4*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1) + 2*q33**2*r1**2*r2**2*math.cos(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1) + 2*q33**2*r1**2*r3**2*math.cos(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1) - 2*q33**2*r1**2*r2**2*math.cos(th1)**4*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1) - 2*q33**2*r2**2*r3**2*math.cos(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1) + 2*q23**2*r2**2*r3**2*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1) - (5919489*q13*q23*math.cos(th1)*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/12500000 - (7923011*q13*q23*r1**2*math.cos(th1)**3*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/25000000 - (3249*q13*q23*r2**2*math.cos(th1)*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)**2)/250000 - (2401*q13*q23*r2**2*math.cos(th1)*math.sin(th1)**3*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/5000 - (4081989*q13*q23*r2**2*math.cos(th1)**3*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/25000000 + (3249*q13*q23*r3**2*math.cos(th1)*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)**2)/250000 - 4*q13*q23*r2**4*math.cos(th1)**3*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1) + 2*q13*q23*r1**4*math.cos(th1)**5*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1) + 2*q13*q23*r2**4*math.cos(th1)**5*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1) - (7923011*q13**2*r1*r2*math.cos(th1)**3*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/25000000 - (7923011*q23**2*r1*r2*math.cos(th1)*math.sin(th1)**3*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/25000000 + (3249*q33**2*r1*r2*math.cos(th1)*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)**2)/250000 + 4*q33**2*r1*r2**3*math.cos(th1)*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1) + 4*q13**2*r2**3*math.cos(th1)**3*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100) - 4*q23**2*r2**3*math.cos(th1)**3*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100) + (15920967*q23*q33*r3*math.cos(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/25000000 + 4*q13**2*r1**2*r2**2*math.cos(th1)**4*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1) + 2*q23**2*r1**2*r2**2*math.cos(th1)**2*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1) + 2*q23**2*r1**2*r3**2*math.cos(th1)**2*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1) + 4*q23**2*r1**2*r2**2*math.cos(th1)**2*math.sin(th1)**4*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1) - 2*q23**2*r1**2*r2**2*math.cos(th1)**4*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1) - 2*q23**2*r2**2*r3**2*math.cos(th1)**2*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1) + 4*q33**2*r1**2*r2**2*math.cos(th1)**2*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1) - (15920967*q13*q33*r3*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/25000000 - (2401*q13*q33*r1*r3*math.cos(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/5000 - (2401*q23*q33*r2*r3*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/5000 + (480200*q13*q23*r1**2*math.cos(th1)**3*math.sin(th1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 + (480200*q13*q23*r2**2*math.cos(th1)*math.sin(th1)**3*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - (9931479237*q13*q23*r3*math.cos(th1)*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/125000000000 + (5841633*q13*q33*r2*math.cos(th1)*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/50000000 + (5841633*q23*q33*r1*math.cos(th1)*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/50000000 - (480200*q13**2*r1*r2*math.cos(th1)*math.sin(th1)**3*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - (480200*q23**2*r1*r2*math.cos(th1)**3*math.sin(th1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - (4081989*q13**2*r1*math.cos(th1)**2*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/25000000 - (4081989*q13**2*r2*math.cos(th1)*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/25000000 + (4081989*q23**2*r1*math.cos(th1)**2*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/25000000 + (4081989*q23**2*r2*math.cos(th1)*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/25000000 - (3249*q13*q23*r1**2*math.cos(th1)**3*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)**2)/250000 + (3249*q13*q23*r2**2*math.cos(th1)**3*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)**2)/250000 - (3249*q13**2*r1*r2*math.cos(th1)**3*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)**2)/250000 + 4*q13**2*r1*r2**3*math.cos(th1)**3*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1) - 4*q13**2*r1*r2**3*math.cos(th1)**5*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1) + 4*q13**2*r1**3*r2*math.cos(th1)**5*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1) - (3249*q23**2*r1*r2*math.cos(th1)*math.sin(th1)**3*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)**2)/250000 + 4*q23**2*r1*r2**3*math.cos(th1)*math.sin(th1)**3*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1) - 4*q33**2*r1*r2**3*math.cos(th1)**3*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1) + 4*q33**2*r1**3*r2*math.cos(th1)**3*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1) + 4*q13**2*r1**2*math.cos(th1)**2*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2 + 4*q23**2*r2**2*math.cos(th1)**2*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2 + (4081989*q13*q23*r1*math.cos(th1)**3*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/25000000 - (2433*q23*q33*r2**2*math.cos(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/2500 - (3249*q23*q33*r3*math.cos(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)**2*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/250000 - (7299*q23*q33*r3**2*math.cos(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/2500 + 4*q23*q33*r3**3*math.cos(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100) - (1947211*q13*q23*r3*math.cos(th1)*math.sin(th1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/27075 - (4081989*q13*q23*r2*math.sin(th1)**3*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/25000000 + (2433*q13*q33*r2**2*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/2500 + (3249*q13*q33*r3*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)**2*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/250000 + (7299*q13*q33*r3**2*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/2500 - 4*q13*q33*r3**3*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100) - (3249*q13*q33*r1*r3*math.cos(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)**2)/125000 - (3249*q23*q33*r2*r3*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)**2)/125000 + (4081989*q13*q23*r2**2*math.cos(th1)*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/25000000 - (7904817*q13*q23*r3*math.cos(th1)*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)**2)/1250000000 + (15920967*q13*q23*r3**2*math.cos(th1)*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/25000000 + 2*q13*q23*r2**4*math.cos(th1)*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1) - (2433*q13*q23*r3**3*math.cos(th1)*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/1250 + 2*q13*q23*r3**4*math.cos(th1)*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1) + (7904817*q13*q33*r2*math.cos(th1)*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)**2)/1250000000 + (7904817*q23*q33*r1*math.cos(th1)*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)**2)/1250000000 + (4081989*q33**2*r1*r2*math.cos(th1)*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/25000000 + (3249*q13**2*r1*math.cos(th1)**2*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)**2*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/250000 + (3249*q13**2*r2*math.cos(th1)*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)**2*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/250000 - 4*q13**2*r2**3*math.cos(th1)*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100) - 4*q13**2*r1**3*math.cos(th1)**4*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100) - (3249*q23**2*r1*math.cos(th1)**2*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)**2*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/250000 - (3249*q23**2*r2*math.cos(th1)*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)**2*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/250000 + 4*q23**2*r2**3*math.cos(th1)*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100) + 4*q23**2*r1**3*math.cos(th1)**4*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100) - (2433*q23**2*r1**2*r3*math.cos(th1)**2*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/2500 + (2433*q23**2*r2**2*r3*math.cos(th1)**2*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/2500 - 4*q23**2*r1*r2**3*math.cos(th1)**3*math.sin(th1)**3*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1) + 4*q23**2*r1**3*r2*math.cos(th1)**3*math.sin(th1)**3*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1) - (3249*q13*q23*r1*math.cos(th1)**3*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)**2*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/250000 + 4*q13*q23*r1**3*math.cos(th1)**5*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100) - (2433*q23*q33*r1**2*math.cos(th1)**3*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/2500 + (2433*q23*q33*r2**2*math.cos(th1)**3*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/2500 + (480200*q13*q23*r3**2*math.cos(th1)*math.sin(th1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 + (3249*q13*q23*r2*math.sin(th1)**3*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)**2*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/250000 - 4*q13*q23*r2**3*math.sin(th1)**3*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100) + (2433*q13*q23*r2*r3*math.sin(th1)**3*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/1250 - 4*q13*q33*r2**2*r3*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100) - (2433*q13*q23*r2**2*r3*math.cos(th1)*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/1250 - (3249*q13*q33*r2*r3*math.cos(th1)*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)**2)/125000 - (3249*q23*q33*r1*r3*math.cos(th1)*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)**2)/125000 - (2433*q33**2*r1*r2*r3*math.cos(th1)*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/1250 - 8*q13*q23*r1**2*math.cos(th1)**3*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2 - 4*q13*q23*r1**3*math.cos(th1)**3*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100) - 8*q13*q23*r2**2*math.cos(th1)*math.sin(th1)**3*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2 + 4*q13*q23*r2**3*math.cos(th1)**2*math.sin(th1)**3*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100) - 4*q13**2*r1*r2**2*math.cos(th1)**2*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100) + 8*q13**2*r1*r2*math.cos(th1)*math.sin(th1)**3*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2 - 4*q13**2*r1*r3**2*math.cos(th1)**2*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100) + 4*q13**2*r1*r2**2*math.cos(th1)**4*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100) - 4*q13**2*r2*r3**2*math.cos(th1)*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100) + 4*q23**2*r1*r2**2*math.cos(th1)**2*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100) + 8*q23**2*r1*r2*math.cos(th1)**3*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2 + 4*q23**2*r1*r3**2*math.cos(th1)**2*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100) - 4*q23**2*r1*r2**2*math.cos(th1)**4*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100) + 4*q23**2*r2*r3**2*math.cos(th1)*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100) + 8*q13*q23*r1**2*r2**2*math.cos(th1)**3*math.sin(th1)**3*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1) + 4*q13*q23*r1*r2**2*math.cos(th1)**3*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100) + 4*q13*q23*r1*r3**2*math.cos(th1)**3*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100) - 4*q13*q23*r1*r2**2*math.cos(th1)**5*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100) + 4*q23*q33*r1**2*r3*math.cos(th1)**3*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100) - 4*q23*q33*r2**2*r3*math.cos(th1)**3*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100) - 4*q13*q23*r2*r3**2*math.sin(th1)**3*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100) - (4081989*q13*q23*r1*math.cos(th1)*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/25000000 + (4081989*q13*q23*r2*math.cos(th1)**2*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/25000000 + (2433*q13*q23*r3*math.cos(th1)*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/625 - (7923011*q13*q23*r1*r2*math.cos(th1)**2*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/12500000 - (2433*q13*q23*r1**2*r3*math.cos(th1)**3*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/1250 + 4*q13*q23*r2**2*r3**2*math.cos(th1)*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1) + (2433*q13*q23*r2**2*r3*math.cos(th1)**3*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/1250 - (2433*q13**2*r1*r2*r3*math.cos(th1)**3*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/1250 - (2433*q23**2*r1*r2*r3*math.cos(th1)*math.sin(th1)**3*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/1250 + 4*q33**2*r1*r2*r3**2*math.cos(th1)*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1) - 8*q13**2*r1*r2**2*math.cos(th1)**2*math.sin(th1)**3*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100) - 12*q13**2*r1**2*r2*math.cos(th1)**3*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100) + 8*q23**2*r1*r2**2*math.cos(th1)**2*math.sin(th1)**3*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100) + 12*q23**2*r1**2*r2*math.cos(th1)**3*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100) + (960400*q13*q23*r1*r2*math.cos(th1)**2*math.sin(th1)**2*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - (2401*q13*q33*r2*r3*math.cos(th1)*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/5000 - (2401*q23*q33*r1*r3*math.cos(th1)*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/5000 + (3249*q13*q23*r1*math.cos(th1)*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)**2*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/250000 - (3249*q13*q23*r2*math.cos(th1)**2*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)**2*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/250000 + 4*q13*q23*r2**3*math.cos(th1)**2*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100) - 8*q13*q23*r3**2*math.cos(th1)*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2 - 4*q13*q23*r2**3*math.cos(th1)**4*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100) + (2433*q13*q33*r1**2*math.cos(th1)**2*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/2500 - (2433*q13*q33*r2**2*math.cos(th1)**2*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/2500 + (2433*q13**2*r1*r3*math.cos(th1)**2*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/1250 + (2433*q13**2*r2*r3*math.cos(th1)*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/1250 - (2433*q23**2*r1*r3*math.cos(th1)**2*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/1250 - (2433*q23**2*r2*r3*math.cos(th1)*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/1250 - (3249*q13*q23*r1*r2*math.cos(th1)**2*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)**2)/125000 + 8*q13*q23*r1*r2**3*math.cos(th1)**2*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1) + 4*q13*q23*r1**2*r2**2*math.cos(th1)**3*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1) + 4*q13*q23*r1**2*r3**2*math.cos(th1)**3*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1) - 8*q13*q23*r1*r2**3*math.cos(th1)**4*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1) - 4*q13*q23*r1**2*r2**2*math.cos(th1)**5*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1) + 8*q13*q23*r1**3*r2*math.cos(th1)**4*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1) - 4*q13*q23*r2**2*r3**2*math.cos(th1)**3*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1) + 4*q13**2*r1*r2*r3**2*math.cos(th1)**3*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1) + 4*q23**2*r1*r2*r3**2*math.cos(th1)*math.sin(th1)**3*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1) - (2433*q13*q23*r1*r3*math.cos(th1)**3*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/1250 + 4*q23*q33*r2**2*r3*math.cos(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100) - 4*q13*q23*r1*r2**2*math.cos(th1)*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100) - 4*q13*q23*r1*r3**2*math.cos(th1)*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100) - 8*q13*q23*r1*r2**2*math.cos(th1)*math.sin(th1)**4*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100) + 4*q13*q23*r2*r3**2*math.cos(th1)**2*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100) + 12*q13*q23*r1**2*r2*math.cos(th1)**4*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100) - 4*q13*q33*r1**2*r3*math.cos(th1)**2*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100) + 4*q13*q33*r2**2*r3*math.cos(th1)**2*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100) + 8*q13*q23*r1*r2*r3**2*math.cos(th1)**2*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1) - 16*q13*q23*r1*r2*math.cos(th1)**2*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2 + 12*q13*q23*r1*r2**2*math.cos(th1)**3*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100) - 12*q13*q23*r1**2*r2*math.cos(th1)**2*math.sin(th1)**3*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100) + (2433*q13*q23*r1*r3*math.cos(th1)*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/1250 - (2433*q13*q23*r2*r3*math.cos(th1)**2*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/1250 + (2433*q13*q33*r1*r2*math.cos(th1)*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/1250 - (2433*q23*q33*r1*r2*math.cos(th1)**2*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/1250 - (2433*q13*q23*r1*r2*r3*math.cos(th1)**2*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/625 - 8*q13*q33*r1*r2*r3*math.cos(th1)*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100) + 8*q23*q33*r1*r2*r3*math.cos(th1)**2*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/((162214160871*q13*q23*math.sin(th1)**2*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/950000000000 - (162214160871*q13*q23*math.cos(th1)**2*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/950000000000 - (16533132777*q23*q33*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/2500000000000 - (6795369*q33**2*r1*math.cos(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/250000000 - (6795369*q33**2*r2*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/250000000 + (162214160871*q13**2*math.cos(th1)*math.sin(th1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/950000000000 - (162214160871*q23**2*math.cos(th1)*math.sin(th1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/950000000000 + (6795369*q13**2*r1*math.cos(th1)**3*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/250000000 + (6795369*q23**2*r2*math.sin(th1)**3*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/250000000 - (16533132777*q13*q33*math.cos(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/2500000000000 - (39739*q13*q23*r2**2*math.cos(th1)**2*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/9500 - (39739*q13*q23*r1**2*math.cos(th1)**4*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/9500 - (119217*q13*q23*r3**2*math.cos(th1)**2*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/9500 + (39739*q13*q23*r2**2*math.cos(th1)**4*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/9500 + (980*q13*q23*r3**3*math.cos(th1)**2*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/57 - (980*q23*q33*r1**3*math.cos(th1)**4*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/57 + (6795369*q13*q23*math.cos(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/500000000 + (39739*q13*q23*r2**2*math.sin(th1)**2*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/9500 + (119217*q13*q23*r3**2*math.sin(th1)**2*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/9500 - (980*q13*q23*r3**3*math.sin(th1)**2*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/57 + (980*q13*q33*r2**3*math.sin(th1)**2*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/57 - (260042461*q13**2*r3*math.cos(th1)*math.sin(th1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/95000000 + (260042461*q23**2*r3*math.cos(th1)*math.sin(th1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/95000000 - (6795369*q13*q23*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/500000000 + (2793*q13*q33*r1**2*math.cos(th1)**3*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/25000 - (2793*q13**2*r1*r3*math.cos(th1)**3*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/25000 + (2793*q23*q33*r2**2*math.sin(th1)**3*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/25000 - (2793*q23**2*r2*r3*math.sin(th1)**3*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/25000 + (6795369*q13*q33*r3*math.cos(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/125000000 + (6795369*q23*q33*r3*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/125000000 + (39739*q13**2*r2**2*math.cos(th1)*math.sin(th1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/9500 + (119217*q13**2*r3**2*math.cos(th1)*math.sin(th1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/9500 - (980*q13**2*r3**3*math.cos(th1)*math.sin(th1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/57 - (39739*q23**2*r2**2*math.cos(th1)*math.sin(th1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/9500 - (119217*q23**2*r3**2*math.cos(th1)*math.sin(th1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/9500 + (980*q23**2*r3**3*math.cos(th1)*math.sin(th1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/57 - (6795369*q13**2*math.cos(th1)*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/500000000 + (6795369*q23**2*math.cos(th1)*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/500000000 + (6795369*q13**2*r2*math.cos(th1)**2*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/250000000 + (6795369*q23**2*r1*math.cos(th1)*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/250000000 + (260042461*q13*q23*r3*math.cos(th1)**2*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/95000000 - (66672487*q23*q33*r1*math.cos(th1)**2*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/95000000 - (260042461*q13*q23*r3*math.sin(th1)**2*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/95000000 + (66672487*q13*q33*r2*math.sin(th1)**2*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/95000000 - (2793*q13*q33*r3**2*math.cos(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/25000 + (2793*q33**2*r1*r3*math.cos(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/25000 - (2793*q23*q33*r3**2*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/25000 + (2793*q33**2*r2*r3*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/25000 + (39739*q13**2*r1**2*math.cos(th1)**3*math.sin(th1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/9500 - (39739*q13**2*r2**2*math.cos(th1)**3*math.sin(th1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/9500 - (39739*q23**2*r1**2*math.cos(th1)**3*math.sin(th1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/9500 + (39739*q23**2*r2**2*math.cos(th1)**3*math.sin(th1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/9500 + (2793*q13**2*r3*math.cos(th1)*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/50000 - (2793*q23**2*r3*math.cos(th1)*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/50000 + (2793*q13*q33*r2**2*math.cos(th1)*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/25000 + (2793*q23*q33*r1**2*math.cos(th1)**2*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/25000 - (2793*q13**2*r2*r3*math.cos(th1)**2*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/25000 - (2793*q23**2*r1*r3*math.cos(th1)*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/25000 + (39739*q23*q33*r1*r3*math.cos(th1)**2*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/4750 - (39739*q13*q33*r2*r3*math.sin(th1)**2*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/4750 + (66672487*q13*q33*r1*math.cos(th1)*math.sin(th1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/95000000 - (66672487*q23*q33*r2*math.cos(th1)*math.sin(th1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/95000000 + (39739*q13*q23*r1**2*math.cos(th1)**2*math.sin(th1)**2*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/9500 - (39739*q13*q23*r2**2*math.cos(th1)**2*math.sin(th1)**2*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/9500 - (980*q13*q33*r2**3*math.cos(th1)**2*math.sin(th1)**2*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/57 + (39739*q13**2*r1*r2*math.cos(th1)**2*math.sin(th1)**2*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/4750 - (980*q13**2*r1**2*r3*math.cos(th1)**3*math.sin(th1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/57 + (980*q13**2*r2**2*r3*math.cos(th1)**3*math.sin(th1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/57 - (39739*q23**2*r1*r2*math.cos(th1)**2*math.sin(th1)**2*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/4750 + (980*q23**2*r1**2*r3*math.cos(th1)**3*math.sin(th1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/57 - (980*q23**2*r2**2*r3*math.cos(th1)**3*math.sin(th1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/57 + (980*q13*q23*r2**2*r3*math.cos(th1)**2*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/57 + (980*q13*q23*r1**2*r3*math.cos(th1)**4*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/57 - (980*q13*q23*r2**2*r3*math.cos(th1)**4*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/57 - (980*q23*q33*r1*r2**2*math.cos(th1)**2*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/57 - (980*q23*q33*r1*r3**2*math.cos(th1)**2*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/57 + (980*q23*q33*r1*r2**2*math.cos(th1)**4*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/57 - (2793*q13*q23*r3*math.cos(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/50000 + (2793*q23*q33*r1*math.cos(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/50000 - (980*q13*q23*r2**2*r3*math.sin(th1)**2*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/57 + (980*q13*q33*r2*r3**2*math.sin(th1)**2*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/57 - (980*q23*q33*r2**3*math.cos(th1)*math.sin(th1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/57 + (2793*q13*q23*r3*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/50000 - (2793*q13*q33*r2*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/50000 + (6795369*q13*q23*r1*math.cos(th1)**2*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/125000000 + (6795369*q13*q23*r2*math.cos(th1)*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/125000000 + (980*q13*q33*r1**3*math.cos(th1)**3*math.sin(th1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/57 + (980*q23*q33*r2**3*math.cos(th1)**3*math.sin(th1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/57 - (980*q13**2*r2**2*r3*math.cos(th1)*math.sin(th1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/57 + (980*q23**2*r2**2*r3*math.cos(th1)*math.sin(th1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/57 + (39739*q13*q23*r1*r2*math.cos(th1)*math.sin(th1)**3*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/4750 - (39739*q13*q23*r1*r2*math.cos(th1)**3*math.sin(th1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/4750 + (980*q13*q33*r1*r2**2*math.cos(th1)*math.sin(th1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/57 + (980*q13*q33*r1*r3**2*math.cos(th1)*math.sin(th1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/57 - (980*q23*q33*r2*r3**2*math.cos(th1)*math.sin(th1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/57 - (2793*q13*q33*r1*math.cos(th1)*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/50000 + (2793*q23*q33*r2*math.cos(th1)*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/50000 - (2793*q13*q23*r1*r3*math.cos(th1)**2*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/12500 - (2793*q13*q23*r2*r3*math.cos(th1)*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/12500 + (2793*q13*q33*r1*r2*math.cos(th1)**2*math.sin(th1)*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/12500 + (2793*q23*q33*r1*r2*math.cos(th1)*math.sin(th1)**2*((1000000*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100)**2)/3249 - 1))/12500 + (1960*q13*q33*r1*r2**2*math.cos(th1)*math.sin(th1)**3*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/57 - (980*q13*q33*r1*r2**2*math.cos(th1)**3*math.sin(th1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/57 - (980*q23*q33*r1**2*r2*math.cos(th1)**3*math.sin(th1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/19 - (39739*q13*q33*r1*r3*math.cos(th1)*math.sin(th1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/4750 + (39739*q23*q33*r2*r3*math.cos(th1)*math.sin(th1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/4750 - (980*q13*q23*r1**2*r3*math.cos(th1)**2*math.sin(th1)**2*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/57 + (980*q13*q23*r2**2*r3*math.cos(th1)**2*math.sin(th1)**2*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/57 + (980*q13*q33*r1**2*r2*math.cos(th1)**2*math.sin(th1)**2*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/19 - (1960*q23*q33*r1*r2**2*math.cos(th1)**2*math.sin(th1)**2*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/57 - (1960*q13**2*r1*r2*r3*math.cos(th1)**2*math.sin(th1)**2*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/57 + (1960*q23**2*r1*r2*r3*math.cos(th1)**2*math.sin(th1)**2*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/57 - (1960*q13*q23*r1*r2*r3*math.cos(th1)*math.sin(th1)**3*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/57 + (1960*q13*q23*r1*r2*r3*math.cos(th1)**3*math.sin(th1)*(r2*math.cos(th1) - r1*math.sin(th1) + 1/100))/57)
    c4   =   (r1*math.sin(th1)-r2*math.cos(th1)+b3-b2)/b5
    th4 = np.append(th4, np.arctan2(s4,c4))
theta4 = th4
q4 = th4 - 0.5*np.pi
# *****************wrapToPi_q4*******************#
xwrap=np.remainder(q4, 2*np.pi)
mask = np.abs(xwrap)>np.pi
xwrap[mask] -= 2*np.pi * np.sign(xwrap[mask])
q4 = xwrap
# *************possible_solution_for_q4_regarding_joint_limitation*************#
pos_sol_q4 = [index for index,value in enumerate(q4) if (value >= -2.60) & (value <= 2.60)]
a_numpy_array = np.array(q4)
accessed_array = a_numpy_array[pos_sol_q4]
q4 = np.array(accessed_array)
a_numpy_array = np.array(theta4)
accessed_array = a_numpy_array[pos_sol_q4]
theta4 = np.array(accessed_array)
a_numpy_array = np.array(theta1)
accessed_array = a_numpy_array[pos_sol_q4]
theta1 = np.array(accessed_array)
#******************************calculate_theta3_2******************************#
th3m2 = np.array([])
for i in range(len(theta1)):
    th1 = theta1[i]
    th4 = theta4[i]
    A11  =  -q33*math.sin(th4);
    A22  =   q13*math.cos(th1)*math.sin(th4) + q23*math.sin(th1)*math.sin(th4);
    A33  =   q13*math.cos(th4)*math.sin(th1) - q23*math.cos(th1)*math.cos(th4);
    B11  =   2*b5*(r3-b1)*math.sin(th4) - 2*b4*(r1*math.cos(th1) + r2*math.sin(th1));
    B22  =  -2*b4*(r3-b1) - 2*b5*math.sin(th4)*(r1*math.cos(th1) + r2*math.sin(th1));
    B33  =   b4**2 + b5**2*math.sin(th4)**2 + (r1*math.cos(th1) + r2*math.sin(th1))**2 + (r3-b1)**2 - a2**2;
    M = np.array([[A11, A22], [B11, B22]])
    N = np.array([-A33, -B33])
    s_c_3_2 = np.linalg.solve(M, N)
    s3_2 = s_c_3_2[0]
    c3_2 = s_c_3_2[1]
    th3m2 = np.append(th3m2, np.arctan2(s3_2,c3_2))
theta3m2 = th3m2
#****************************calculate_theta_5*********************************#
th5 = np.array([])
for i in range(len(theta1)):
    th1 = theta1[i]
    th4 = theta4[i]
    th3m2 = theta3m2[i]
    c5  =  -(q13*math.cos(th1)*math.sin(th3m2) + q23*math.sin(th1)*math.sin(th3m2) + q33*math.cos(th3m2))
    s5  =   (-q13*math.sin(th1) + q23*math.cos(th1))/math.sin(th4)
    th5 = np.append(th5, np.arctan2(s5,c5))
theta5 = th5
q5 = th5 - np.pi
# **************wrapToPi q5*****************#
xwrap=np.remainder(q5, 2*np.pi)
mask = np.abs(xwrap)>np.pi
xwrap[mask] -= 2*np.pi * np.sign(xwrap[mask])
q5 = xwrap
# ************possible_solution_for_q5_regarding_joint_limitation**************#
pos_sol_q5 = [index for index,value in enumerate(q5) if (value >= -2.53) & (value <= 2.53)]
a_numpy_array = np.array(q5)
accessed_array = a_numpy_array[pos_sol_q5]
q5 = np.array(accessed_array)
a_numpy_array = np.array(theta4)
accessed_array = a_numpy_array[pos_sol_q5]
theta4 = np.array(accessed_array)
a_numpy_array = np.array(theta1)
accessed_array = a_numpy_array[pos_sol_q5]
theta1 = np.array(accessed_array)
a_numpy_array = np.array(theta3m2)
accessed_array = a_numpy_array[pos_sol_q5]
theta3m2 = np.array(accessed_array)
a_numpy_array = np.array(theta5)
accessed_array = a_numpy_array[pos_sol_q5]
theta5 = np.array(accessed_array)
#**************************calculate_theta_2***********************************#
th2 = np.array([])
for i in range(len(theta1)):
    th1 = theta1[i]
    th4 = theta4[i]
    th3m2 = theta3m2[i]
    c2  =  ((r1*math.cos(th1) + r2*math.sin(th1)) - (b5*math.cos(th3m2)*math.sin(th4) + b4*math.sin(th3m2)))/a2
    s2  =  ((r3 - b1) + b5*math.sin(th3m2)*math.sin(th4) - b4*math.cos(th3m2))/a2
    th2 = np.append(th2, np.arctan2(s2,c2))
theta2 = th2
q2 = th2 - 0.5*np.pi
# *************wrapToPi q4****************#
xwrap=np.remainder(q2, 2*np.pi)
mask = np.abs(xwrap)>np.pi
xwrap[mask] -= 2*np.pi * np.sign(xwrap[mask])
q2 = xwrap
# ************possible_solution_for_q2_regarding_joint_limitation**************#
pos_sol_q2 = [index for index,value in enumerate(q2) if (value >= -2.61) & (value <= 2.61)]
a_numpy_array = np.array(q2)
accessed_array = a_numpy_array[pos_sol_q2]
q2 = np.array(accessed_array)
a_numpy_array = np.array(theta4)
accessed_array = a_numpy_array[pos_sol_q2]
theta4 = np.array(accessed_array)
a_numpy_array = np.array(theta1)
accessed_array = a_numpy_array[pos_sol_q2]
theta1 = np.array(accessed_array)
a_numpy_array = np.array(theta3m2)
accessed_array = a_numpy_array[pos_sol_q2]
theta3m2 = np.array(accessed_array)
a_numpy_array = np.array(theta5)
accessed_array = a_numpy_array[pos_sol_q2]
theta5 = np.array(accessed_array)
a_numpy_array = np.array(theta2)
accessed_array = a_numpy_array[pos_sol_q2]
theta2 = np.array(accessed_array)
theta3 = theta3m2 + theta2
#**************************calculate_theta_6***********************************#
th6 = np.array([])
for i in range(len(theta1)):
    th1 = theta1[i]
    th5 = theta5[i]
    th3m2 = theta3m2[i]
    c6  =   (q11*math.cos(th1)*math.sin(th3m2) + q21*math.sin(th1)*math.sin(th3m2) + q31*math.cos(th3m2))/math.sin(th5)
    s6  =  -(q12*math.cos(th1)*math.sin(th3m2) + q22*math.sin(th1)*math.sin(th3m2) + q32*math.cos(th3m2))/math.sin(th5)
    th6 = np.append(th6, np.arctan2(s6,c6))
theta6 = th6
q6 = th6 - 0.5*np.pi
# ********wrapToPi q6*********
xwrap=np.remainder(q6, 2*np.pi)
mask = np.abs(xwrap)>np.pi
xwrap[mask] -= 2*np.pi * np.sign(xwrap[mask])
q6 = xwrap
# ***********possible_solution_for-q6_regarding_joint_limitation***************#
pos_sol_q6 = [index for index,value in enumerate(q6) if (value >= -2.60) & (value <= 2.60)]
a_numpy_array = np.array(q6)
accessed_array = a_numpy_array[pos_sol_q6]
q6 = np.array(accessed_array)
a_numpy_array = np.array(theta4)
accessed_array = a_numpy_array[pos_sol_q6]
theta4 = np.array(accessed_array)
a_numpy_array = np.array(theta1)
accessed_array = a_numpy_array[pos_sol_q6]
theta1 = np.array(accessed_array)
a_numpy_array = np.array(theta3m2)
accessed_array = a_numpy_array[pos_sol_q6]
theta3m2 = np.array(accessed_array)
a_numpy_array = np.array(theta5)
accessed_array = a_numpy_array[pos_sol_q6]
theta5 = np.array(accessed_array)
a_numpy_array = np.array(theta2)
accessed_array = a_numpy_array[pos_sol_q6]
theta2 = np.array(accessed_array)
a_numpy_array = np.array(theta3)
accessed_array = a_numpy_array[pos_sol_q6]
theta3 = np.array(accessed_array)
a_numpy_array = np.array(theta6)
accessed_array = a_numpy_array[pos_sol_q6]
theta6 = np.array(accessed_array)
# *************************************************************
q1 = theta1 - 0
q2 = theta2 - 0.5*np.pi
q3 = theta3 - 0.5*np.pi
q4 = theta4 - 0.5*np.pi
q5 = theta5 - np.pi
q6 = theta6 - 0.5*np.pi
# *****************wrapToPi******************#
xwrap=np.remainder(q1, 2*np.pi)
mask = np.abs(xwrap)>np.pi
xwrap[mask] -= 2*np.pi * np.sign(xwrap[mask])
q1_final = xwrap
print("q1_final =", q1_final)
xwrap=np.remainder(q2, 2*np.pi)
mask = np.abs(xwrap)>np.pi
xwrap[mask] -= 2*np.pi * np.sign(xwrap[mask])
q2_final = xwrap
print("q2_final =", q2_final)
xwrap=np.remainder(q3, 2*np.pi)
mask = np.abs(xwrap)>np.pi
xwrap[mask] -= 2*np.pi * np.sign(xwrap[mask])
q3_final = xwrap
print("q3_final =", q3_final)
xwrap=np.remainder(q4, 2*np.pi)
mask = np.abs(xwrap)>np.pi
xwrap[mask] -= 2*np.pi * np.sign(xwrap[mask])
q4_final = xwrap
print("q4_final =", q4_final)
xwrap=np.remainder(q5, 2*np.pi)
mask = np.abs(xwrap)>np.pi
xwrap[mask] -= 2*np.pi * np.sign(xwrap[mask])
q5_final = xwrap
print("q5_final =", q5_final)
xwrap=np.remainder(q6, 2*np.pi)
mask = np.abs(xwrap)>np.pi
xwrap[mask] -= 2*np.pi * np.sign(xwrap[mask])
q6_final = xwrap
print("q6_final =", q6_final)
#**************************select_optimum_solution*****************************#
q_init = np.array([q1_init, q2_init, q3_init, q4_init, q5_init, q6_init])
norm_movement = np.array([])
for i in range(len(q1_final)):
    q_final = np.array([q1_final[i], q2_final[i], q3_final[i], q4_final[i], q5_final[i], q6_final[i]])
    diff = q_final - q_init
    norm_movement = np.append(norm_movement, LA.norm(diff))
index_min = np.argmin(norm_movement)
q_final = np.array([q1_final[index_min], q2_final[index_min], q3_final[index_min], q4_final[index_min], q5_final[index_min], q6_final[index_min]])
#**************************print_possible_solutions****************************#
print("q_final =", q_final)
# -------- JOINT_1_MOVEMENT --------#
#SimJointMove('/my_gen3_lite/joint_1_position_controller/command', q1_final[index_min])
#sleep(3.0)
# -------- JOINT_2_MOVEMENT --------#
#SimJointMove('/my_gen3_lite/joint_2_position_controller/command', q2_final[index_min])
#sleep(3.0)
# -------- JOINT_3_MOVEMENT --------#
#SimJointMove('/my_gen3_lite/joint_3_position_controller/command', q3_final[index_min])
#sleep(3.0)
# -------- JOINT_4_MOVEMENT --------#
#SimJointMove('/my_gen3_lite/joint_4_position_controller/command', q4_final[index_min])
#sleep(3.0)
# -------- JOINT 5 MOVEMENT --------#
#SimJointMove('/my_gen3_lite/joint_5_position_controller/command', q5_final[index_min])
#sleep(3.0)
# -------- JOINT 6 MOVEMENT --------#
#SimJointMove('/my_gen3_lite/joint_6_position_controller/command', q6_final[index_min])
#sleep(3.0)
#***************** Plot_end_effector_position by forward_kinematics*****************#
a = np.array([0, 0.28, 0, 0, 0, 0])
b = np.array([0.2433, 0.03, 0.02, 0.245, 0.057, 0.235])
al = np.array([0.5*math.pi, math.pi, 0.5*math.pi, 0.5*math.pi, 0.5*math.pi, 0])
th = np.array([q1_final[index_min], q2_final[index_min], q3_final[index_min], q4_final[index_min], q5_final[index_min], q6_final[index_min]]) + np.array([0, 0.5*math.pi, 0.5*math.pi, 0.5*math.pi, math.pi, 0.5*math.pi])
Q_1 = np.array([[math.cos(th[0]), -math.cos(al[0])*math.sin(th[0]), math.sin(al[0])*math.sin(th[0])],
                [math.sin(th[0]),  math.cos(al[0])*math.cos(th[0]),-math.sin(al[0])*math.cos(th[0])],
                [              0,                 math.sin(al[0]),               math.cos(al[0])]])

Q_2 = np.array([[math.cos(th[1]), -math.cos(al[1])*math.sin(th[1]), math.sin(al[1])*math.sin(th[1])],
                [math.sin(th[1]),  math.cos(al[1])*math.cos(th[1]),-math.sin(al[1])*math.cos(th[1])],
                [              0,                 math.sin(al[1]),               math.cos(al[1])]])

Q_3 = np.array([[math.cos(th[2]), -math.cos(al[2])*math.sin(th[2]), math.sin(al[2])*math.sin(th[2])],
                [math.sin(th[2]),  math.cos(al[2])*math.cos(th[2]),-math.sin(al[2])*math.cos(th[2])],
                [              0,                 math.sin(al[2]),               math.cos(al[2])]])

Q_4 = np.array([[math.cos(th[3]), -math.cos(al[3])*math.sin(th[3]), math.sin(al[3])*math.sin(th[3])],
                [math.sin(th[3]),  math.cos(al[3])*math.cos(th[3]),-math.sin(al[3])*math.cos(th[3])],
                [              0,                 math.sin(al[3]),               math.cos(al[3])]])

Q_5 = np.array([[math.cos(th[4]), -math.cos(al[4])*math.sin(th[4]), math.sin(al[4])*math.sin(th[4])],
                [math.sin(th[4]),  math.cos(al[4])*math.cos(th[4]),-math.sin(al[4])*math.cos(th[4])],
                [              0,                 math.sin(al[4]),               math.cos(al[4])]])

Q_6 = np.array([[math.cos(th[5]), -math.cos(al[5])*math.sin(th[5]), math.sin(al[5])*math.sin(th[5])],
                [math.sin(th[5]),  math.cos(al[5])*math.cos(th[5]),-math.sin(al[5])*math.cos(th[5])],
                [              0,                 math.sin(al[5]),               math.cos(al[5])]])

a_1 = np.array([[a[0]*math.cos(th[0])],
                [a[0]*math.sin(th[0])],
 	            [               b[0]]])
a_2 = np.array([[a[1]*math.cos(th[1])],
                [a[1]*math.sin(th[1])],
 	            [               b[1]]])
a_3 = np.array([[a[2]*math.cos(th[2])],
                [a[2]*math.sin(th[2])],
 	            [               b[2]]])
a_4 = np.array([[a[3]*math.cos(th[3])],
                [a[3]*math.sin(th[3])],
 	            [               b[3]]])
a_5 = np.array([[a[4]*math.cos(th[4])],
                [a[4]*math.sin(th[4])],
 	            [               b[4]]])
a_6 = np.array([[a[5]*math.cos(th[5])],
                [a[5]*math.sin(th[5])],
 	            [               b[5]]])
l1 = a_1
l2 = a_1 + np.matmul(Q_1, a_2)
l3 = a_1 + np.matmul(Q_1, a_2) +np.matmul(np.matmul(Q_1, Q_2), a_3)
l4 = a_1 + np.matmul(Q_1, a_2) +np.matmul(np.matmul(Q_1, Q_2), a_3) + np.matmul(np.matmul(np.matmul(Q_1, Q_2) , Q_3), a_4)
l5 = a_1 + np.matmul(Q_1, a_2) +np.matmul(np.matmul(Q_1, Q_2), a_3) + np.matmul(np.matmul(np.matmul(Q_1, Q_2) , Q_3), a_4) + np.matmul(np.matmul(np.matmul(np.matmul(Q_1, Q_2) , Q_3), Q_4), a_5)
l6 = a_1 + np.matmul(Q_1, a_2) +np.matmul(np.matmul(Q_1, Q_2), a_3) + np.matmul(np.matmul(np.matmul(Q_1, Q_2) , Q_3), a_4) + np.matmul(np.matmul(np.matmul(np.matmul(Q_1, Q_2) , Q_3), Q_4), a_5) + np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(Q_1, Q_2) , Q_3), Q_4), Q_5), a_6)
P  = a_1 + np.matmul(Q_1, a_2) +np.matmul(np.matmul(Q_1, Q_2), a_3) + np.matmul(np.matmul(np.matmul(Q_1, Q_2) , Q_3), a_4) + np.matmul(np.matmul(np.matmul(np.matmul(Q_1, Q_2) , Q_3), Q_4), a_5) + np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(Q_1, Q_2) , Q_3), Q_4), Q_5), a_6)
R  = np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(Q_1, Q_2) , Q_3), Q_4), Q_5), Q_6)
pitch = np.arcsin(-R[2][0])
roll  = np.arctan2(R[2][1]/math.cos(pitch),R[2][2]/math.cos(pitch))
yaw   = np.arctan2(R[1][0]/math.cos(pitch),R[0][0]/math.cos(pitch))
orientation = np.array([roll, pitch, yaw])
O = np.array([[0, l1[0][0], l2[0][0], l3[0][0], l4[0][0], l5[0][0], l6[0][0]],
              [0, l1[1][0], l2[1][0], l3[1][0], l4[1][0], l5[1][0], l6[1][0]],
              [0, l1[2][0], l2[2][0], l3[2][0], l4[2][0], l5[2][0], l6[2][0]]])
ax = plt.axes(projection='3d')
plt.figure(1)
ax.plot3D(O[0], O[1], O[2], 'green')
label_1 = 'cartesian_position_by_IK = (%f, %f, %f), ' % (O[0][6], O[1][6], O[2][6])
ax.text(O[0][6], O[1][6], O[2][6] - 0.1, label_1, color='green')
label_2 = 'orientation_by_IK = (%f, %f, %f), ' % (roll, pitch, yaw)
ax.text(O[0][6], O[1][6], O[2][6] - 0.2, label_2, color='blue')
point1 = [0, 0, 0]
point2 = [X, Y, Z]
x_values = [point1[0], point2[0]]
y_values = [point1[1], point2[1]]
z_values = [point1[2], point2[2]]
ax.plot3D(x_values, y_values, z_values, 'red')
label_3 = 'desired_position = (%f, %f, %f), ' % (X, Y, Z)
ax.text(X, Y, Z - 0.3, label_3, color='red')
label_4 = 'desired_orientation = (%f, %f, %f), ' % (exm[3], exm[4], exm[5])
ax.text(X, Y, Z - 0.4, label_4, color='red')
ax.set_xlim(-1, 1)
ax.set_ylim(-1, 1)
ax.set_zlim(-1, 1)
ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')
ax.set_zlabel('Z axis')
plt.title('Validation of IK solution by FK')
plt.show()
#*******************reading_joint_states*****************************#
# def joint_state_callback(msg):
#     for i in range(len(msg.position)):
#         rospy.loginfo("I heard %s", msg.position[i])
#
# join_state_sub = rospy.Subscriber("/my_gen3_lite/joint_states", JointState, joint_state_callback)
#
# rospy.spin()