
import numpy as np
import pickle



m1 = 0.9597
m2 = 1.1776
m3 = 0.59768
m4 = 0.52693
m5 = 0.58097
m6 = 0.2018

with open('TB.pkl', 'rb') as h:
    Tb1, Tb2, Tb3, Tb4, Tb5, Tb6 = pickle.load(h)

with open('Inertia.pkl', 'rb') as h:
    I1, I2, I3, I4, I5, I6 = pickle.load(h)


class Dynamics():
    def __init__(self):
        self.Tb1, self.Tb2, self.Tb3, self.Tb4, self.Tb5, self.Tb6 = Tb1, Tb2, Tb3, Tb4, Tb5, Tb6
        self.m1, self.m2, self.m3, self.m4, self.m5, self.m6 = m1, m2, m3, m4, m5, m6
        self.I1, self.I2, self.I3, self.I4, self.I5, self.I6 = I1, I2, I3, I4, I5, I6

    def forward(self, q):
        self.T01 = np.array([[np.cos(q[0]), -np.sin(q[0]), 0, 0],
               [np.sin(q[0]), np.cos(q[0]), 0, 0],
               [0, 0, 1, 0.1283],
               [0, 0, 0, 1]])
        self.T12 = np.array([[np.cos(q[1]), -np.sin(q[1]), 0, 0],
               [0, 0, -1, -0.03],
               [np.sin(q[1]), np.cos(q[1]), 0, 0.115],
               [0, 0, 0, 1]])
        self.T23 = np.array([[np.cos(q[2]), -np.sin(q[2]), 0, 0],
               [-np.sin(q[2]), -np.cos(q[2]), 0, 0.28],
               [0, 0, -1, 0],
               [0, 0, 0, 1]])
        self.T34 = np.array([[np.cos(q[3]), -np.sin(q[3]), 0, 0],
               [0, 0, -1, -0.14],
               [np.sin(q[3]), np.cos(q[3]), 0, 0.02],
               [0, 0, 0, 1]])
        self.T45 = np.array([[0, 0, -1, 0.0285],
               [np.sin(q[4]), np.cos(q[4]), 0, 0],
               [np.cos(q[4]), -np.sin(q[4]), 0, 0.105],
               [0, 0, 0, 1]])
        self.T56 = np.array([[0, 0, -1, -0.105],
               [np.sin(q[5]), np.cos(q[5]), 0, 0],
               [np.cos(q[5]), -np.sin(q[5]), 0, 0.0285],
               [0, 0, 0, 1]])
        self.T01d1 = np.array([[-np.sin(q[0]), -np.cos(q[0]), 0, 0],
                 [np.cos(q[0]), -np.sin(q[0]), 0, 0],
                 [0, 0, 0, 0],
                 [0, 0, 0, 0]])
        self.T12d2 = np.array([[-np.sin(q[1]), -np.cos(q[1]), 0, 0],
                 [0, 0, 0, 0],
                 [np.cos(q[1]), -np.sin(q[1]), 0, 0],
                 [0, 0, 0, 0]])
        self.T23d3 = np.array([[-np.sin(q[2]), -np.cos(q[2]), 0, 0],
                 [-np.cos(q[2]), np.sin(q[2]), 0, 0],
                 [0, 0, 0, 0],
                 [0, 0, 0, 0]])
        self.T34d4 = np.array([[-np.sin(q[3]), -np.cos(q[3]), 0, 0],
                 [0, 0, 0, 0],
                 [np.cos(q[3]), -np.sin(q[3]), 0, 0],
                 [0, 0, 0, 0]])
        self.T45d5 = np.array([[0, 0, 0, 0],
                 [np.cos(q[4]), -np.sin(q[4]), 0, 0],
                 [-np.sin(q[4]), -np.cos(q[4]), 0, 0],
                 [0, 0, 0, 0]])
        self.T56d6 = np.array([[0, 0, 0, 0],
                 [np.cos(q[5]), -np.sin(q[5]), 0, 0],
                 [-np.sin(q[5]), -np.cos(q[5]), 0, 0],
                 [0, 0, 0, 0]])

    def gravity(self):
        g0 = self.m1 * self.T01d1 @ self.Tb1 + self.m2 * self.T01d1 @ self.T12 @ self.Tb2 + \
             self.m3 * self.T01d1 @ self.T12 @ self.T23 @ self.Tb3 + \
             self.m4 * self.T01d1 @ self.T12 @ self.T23 @ self.T34 @ self.Tb4 + \
             self.m5 * self.T01d1 @ self.T12 @ self.T23 @ self.T34 @ self.T45 @ self.Tb5 + \
             self.m6 * self.T01d1 @ self.T12 @ self.T23 @ self.T34 @ self.T45 @ self.T56 @ self.Tb6

        g1 = self.m2 * self.T01 @ self.T12d2 @ self.Tb2 + \
             self.m3 * self.T01 @ self.T12d2 @ self.T23 @ self.Tb3 + \
             self.m4 * self.T01 @ self.T12d2 @ self.T23 @ self.T34 @ self.Tb4 + \
             self.m5 * self.T01 @ self.T12d2 @ self.T23 @ self.T34 @ self.T45 @ self.Tb5 + \
             self.m6 * self.T01 @ self.T12d2 @ self.T23 @ self.T34 @ self.T45 @ self.T56 @ self.Tb6

        g2 = self.m3 * self.T01 @ self.T12 @ self.T23d3 @ self.Tb3 + \
             self.m4 * self.T01 @ self.T12 @ self.T23d3 @ self.T34 @ self.Tb4 +\
             self.m5 * self.T01 @ self.T12 @ self.T23d3 @ self.T34 @ self.T45 @ self.Tb5 + \
             self.m6 * self.T01 @ self.T12 @ self.T23d3 @ self.T34 @ self.T45 @ self.T56 @ self.Tb6

        g3 = self.m4 * self.T01 @ self.T12 @ self.T23 @ self.T34d4 @ self.Tb4 + \
             self.m5 * self.T01 @ self.T12 @ self.T23 @ self.T34d4 @ self.T45 @ self.Tb5 + \
             self.m6 * self.T01 @ self.T12 @ self.T23 @ self.T34d4 @ self.T45 @ self.T56 @ self.Tb6

        g4 = self.m5 * self.T01 @ self.T12 @ self.T23 @ self.T34 @ self.T45d5 @ self.Tb5 + \
             self.m6 * self.T01 @ self.T12 @ self.T23 @ self.T34 @ self.T45d5 @ self.T56 @ self.Tb6

        g5 = self.m6 * self.T01 @ self.T12 @ self.T23 @ self.T45 @ self.T56d6 @ self.Tb6

        return 9.81 * np.array([g0[2, 3], g1[2, 3], g2[2, 3], g3[2, 3], g4[2, 3], g5[2, 3]])


    def Inertia(self):
        self.R12, self.R23, self.R34, self.R45, self.R56 = self.T12[:3, :3], self.T23[:3, :3], self.T34[:3, :3], self.T45[:3, :3], self.T56[:3, :3]
