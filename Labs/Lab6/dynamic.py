
import numpy as np
import pickle



m1 = 0.9597
m2 = 1.1776
m3 = 0.59768
m4 = 0.52693
m5 = 0.58097
m6 = 0.2018
w101 = np.array([0, 0, 1]).T
w212 = np.array([0, 0, 1]).T
w323 = np.array([0, 0, 1]).T
w434 = np.array([0, 0, 1]).T
w545 = np.array([0, 0, 1]).T
w656 = np.array([0, 0, 1]).T

B1 = np.array([0.000025, 0.022135, 0.099377]).T
B2 = np.array([0.029983, 0.21155, 0.045303]).T
B3 = np.array([0.030156, 0.095022, 0.007356]).T
B4 = np.array([0.005752, 0.010004, 0.087192]).T
B5 = np.array([0.080565, 0.009804, 0.018728]).T
B6 = np.array([0.00993, 0.00995, 0.06136]).T

C11 = np.array([0, -0.03, 0.115]).T
C22 = np.array([0, -0.28, 0]).T
C33 = np.array([0, -0.14, 0.02]).T
C44 = np.array([0.0285, 0, 0.105]).T
C55 = np.array([-0.105, 0, 0.0285]).T

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
        q = np.deg2rad(q)
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

    def gravity(self, q):
        '''

        G = sum(m_i * J_li.T) * g

        '''
        self.forward(q)

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

    def inertia(self,q):

        self.forward(q)
        self.R12, self.R23, self.R34, self.R45, self.R56 = self.T12[:3, :3],\
                                                           self.T23[:3, :3],\
                                                           self.T34[:3, :3],\
                                                           self.T45[:3, :3],\
                                                           self.T56[:3, :3]

        self.J1 = np.zeros((6,6))
        self.J2 = np.zeros((6,6))
        self.J3 = np.zeros((6,6))
        self.J4 = np.zeros((6,6))
        self.J5 = np.zeros((6,6))
        self.J6 = np.zeros((6,6))

        ## TODO: We can also use the symbolic solution
        self.J1[0:3,0] = w101
        self.J2[0:3,0] = self.R12.T @ w101
        self.J3[0:3,0] = self.R23.T @  self.R12.T @ w101
        self.J4[0:3,0] = self.R34.T @ self.R23.T @ self.R12.T @ w101
        self.J5[0:3,0] = self.R45.T @ self.R34.T @ self.R23.T @ self.R12.T @ w101
        self.J6[0:3,0] = self.R56.T @ self.R45.T @ self.R34.T @ self.R23.T @ self.R12.T @ w101

        self.J2[0:3,1] = w212
        self.J3[0:3,1] = self.R23.T @ w212
        self.J4[0:3,1] = self.R34.T @  self.R23.T @ w212
        self.J5[0:3,1] = self.R45.T @ self.R34.T @ self.R23.T @ w212
        self.J6[0:3,1] = self.R56.T @ self.R45.T @ self.R34.T @ self.R23.T @ w212

        self.J3[0:3,2] = w323
        self.J4[0:3,2] = self.R34.T @ w323
        self.J5[0:3,2] = self.R45.T @  self.R34.T @ w323
        self.J6[0:3,2] = self.R56.T @ self.R45.T @ self.R34.T @ w323

        self.J4[0:3,3] = w434
        self.J5[0:3,3] = self.R45.T @ w434
        self.J6[0:3,3] = self.R56.T @  self.R45.T @ w434

        self.J5[0:3,4] = w545
        self.J6[0:3,4] = self.R56.T @ w545

        self.J6[0:3,5] = w656

        self.J1[3:6,0] = np.cross(w101, B1)
        self.J2[3:6,0] = np.cross(self.R12.T @ w101, self.R12.T @ C11+B2)
        self.J3[3:6,0] = np.cross(self.R23.T @ self.R12.T @w101,self.R23.T @ self.R12.T @C11+self.R23.T @ C22+B3)
        self.J4[3:6,0] = np.cross(self.R34.T @ self.R23.T @self.R12.T@w101 ,self.R34.T@self.R23.T@self.R12.T@C11 + self.R34.T@ self.R23.T@C22+self.R34.T@C33+B4)
        self.J5[3:6,0] = np.cross(self.R45.T@self.R34.T@self.R23.T@self.R12.T@w101,self.R45.T@self.R34.T@self.R23.T@self.R12.T@C11+self.R45.T@self.R34.T@self.R23.T@C22+self.R45.T@self.R34.T@C33+self.R45.T@C44+B5)
        self.J6[3:6,0] = np.cross(self.R56.T@self.R45.T@self.R34.T@self.R23.T@self.R12.T@w101,self.R56.T@self.R45.T@self.R34.T@self.R23.T@self.R12.T@C11+self.R56.T@self.R45.T@self.R34.T@self.R23.T@C22+self.R56.T@self.R45.T@self.R34.T@C33+self.R56.T@self.R45.T@C44+self.R56.T@C55+B6)

        self.J2[3:6,1] = np.cross(w212, B2)
        self.J3[3:6,1] = np.cross(self.R23.T@w212,self.R23.T@C22+B3)
        self.J4[3:6,1] = np.cross(self.R34.T@self.R23.T@w212,self.R34.T@self.R23.T@C22+self.R34.T@C33+B4)
        self.J5[3:6,1] = np.cross(self.R45.T@self.R34.T@self.R23.T@w212,self.R45.T@self.R34.T@self.R23.T@C22+self.R45.T@self.R34.T@C33+self.R45.T@C44+B5)
        self.J6[3:6,1] = np.cross(self.R56.T@self.R45.T@self.R34.T@self.R23.T@w212,self.R56.T@self.R45.T@self.R34.T@self.R23.T@C22+self.R56.T@self.R45.T@self.R34.T@C33+self.R56.T@self.R45.T@C44+self.R56.T@C55+B6)

        self.J3[3:6,2] = np.cross(w323, B3)
        self.J4[3:6,2] = np.cross(self.R34.T@w323,self.R34.T@C33+B4)
        self.J5[3:6,2] = np.cross(self.R45.T@self.R34.T@w323,self.R45.T@self.R34.T@C33+self.R45.T@C44+B5)
        self.J6[3:6,2] = np.cross(self.R56.T@self.R45.T@self.R34.T@w323,self.R56.T@self.R45.T@self.R34.T@C33+self.R56.T@self.R45.T@C44+self.R56.T@C55+B6)

        self.J4[3:6,3] = np.cross(w434,B4)
        self.J5[3:6,3] = np.cross(self.R45.T@w434,self.R45.T@C44+B5)
        self.J6[3:6,3] = np.cross(self.R56.T@self.R45.T@w434,self.R56.T@self.R45.T@C44+self.R56.T@C55+B6)

        self.J5[3:6,4] = np.cross(w545,B5)
        self.J6[3:6,4] = np.cross(self.R56.T@w545,self.R56.T@C55+B6)

        self.J6[3:6,5] = np.cross(w656,B6)

        return self.J1.T@self.I1@self.J1+self.J2.T@self.I2@self.J2+self.J3.T@self.I3@self.J3+self.J4.T@self.I4@self.J4+self.J5.T@self.I5@self.J5+self.J6.T@self.I6@self.J6

    def coriolis(self, q, qp):

        self.forward(q)

        self.R12d2, self.R23d3, self.R34d4, self.R45d5, self.R56d6 = self.T12d2[:3, :3],\
                                                           self.T23d3[:3, :3],\
                                                           self.T34d4[:3, :3],\
                                                           self.T45d5[:3, :3],\
                                                           self.T56d6[:3, :3]

        self.J2d2 = np.zeros((6,6))
        self.J3d2 = np.zeros((6,6))
        self.J4d2 = np.zeros((6,6))
        self.J5d2 = np.zeros((6,6))
        self.J6d2 = np.zeros((6,6))

        ## TODO: We can also use the symbolic solution
        self.J2d2[0:3,0] = self.R12d2.T @ w101
        self.J3d2[0:3,0] = self.R23.T @  self.R12d2.T @ w101
        self.J4d2[0:3,0] = self.R34.T @ self.R23.T @ self.R12d2.T @ w101
        self.J5d2[0:3,0] = self.R45.T @ self.R34.T @ self.R23.T @ self.R12d2.T @ w101
        self.J6d2[0:3,0] = self.R56.T @ self.R45.T @ self.R34.T @ self.R23.T @ self.R12d2.T @ w101

        self.J2d2[3:6,0] = np.cross(self.R12d2.T @ w101, self.R12.T @ C11+B2) \
                         + np.cross(self.R12.T @ w101, self.R12d2.T @ C11+B2)
        self.J3d2[3:6,0] = np.cross(self.R23.T @ self.R12d2.T @w101,self.R23.T @ self.R12.T @C11+self.R23.T @ C22+B3) \
                         + np.cross(self.R23.T @ self.R12.T @w101,self.R23.T @ self.R12d2.T @C11)
        self.J4d2[3:6,0] = np.cross(self.R34.T @ self.R23.T @self.R12d2.T@w101 ,self.R34.T@self.R23.T@self.R12.T@C11 + self.R34.T@ self.R23.T@C22+self.R34.T@C33+B4) \
                         + np.cross(self.R34.T @ self.R23.T @self.R12.T@w101 ,self.R34.T@self.R23.T@self.R12d2.T@C11)
        self.J5d2[3:6,0] = np.cross(self.R45.T@self.R34.T@self.R23.T@self.R12d2.T@w101,self.R45.T@self.R34.T@self.R23.T@self.R12.T@C11+self.R45.T@self.R34.T@self.R23.T@C22+self.R45.T@self.R34.T@C33+self.R45.T@C44+B5) \
                         + np.cross(self.R45.T@self.R34.T@self.R23.T@self.R12.T@w101,self.R45.T@self.R34.T@self.R23.T@self.R12d2.T@C11)
        self.J6d2[3:6,0] = np.cross(self.R56.T@self.R45.T@self.R34.T@self.R23.T@self.R12d2.T@w101,self.R56.T@self.R45.T@self.R34.T@self.R23.T@self.R12.T@C11+self.R56.T@self.R45.T@self.R34.T@self.R23.T@C22+self.R56.T@self.R45.T@self.R34.T@C33+self.R56.T@self.R45.T@C44+self.R56.T@C55+B6) \
                           + np.cross(self.R56.T@self.R45.T@self.R34.T@self.R23.T@self.R12.T@w101,self.R56.T@self.R45.T@self.R34.T@self.R23.T@self.R12d2.T@C11)


        self.J3d3 = np.zeros((6,6))
        self.J4d3 = np.zeros((6,6))
        self.J5d3 = np.zeros((6,6))
        self.J6d3 = np.zeros((6,6))

        self.J3d3[0:3,0] = self.R23d3.T @  self.R12.T @ w101
        self.J4d3[0:3,0] = self.R34.T @ self.R23d3.T @ self.R12.T @ w101
        self.J5d3[0:3,0] = self.R45.T @ self.R34.T @ self.R23d3.T @ self.R12.T @ w101
        self.J6d3[0:3,0] = self.R56.T @ self.R45.T @ self.R34.T @ self.R23d3.T @ self.R12.T @ w101

        self.J3d3[0:3,1] = self.R23d3.T @ w212
        self.J4d3[0:3,1] = self.R34.T @ self.R23d3.T  @ w212
        self.J5d3[0:3,1] = self.R45.T @ self.R34.T @ self.R23d3.T  @ w212
        self.J6d3[0:3,1] = self.R56.T @ self.R45.T @ self.R34.T @ self.R23d3.T @ w212

        self.J3d3[3:6, 0] = np.cross(self.R23d3.T @ self.R12.T @ w101,
                                     self.R23.T @ self.R12.T @ C11 + self.R23.T @ C22 + B3) + np.cross(
            self.R23.T @ self.R12.T @ w101, self.R23d3.T @ self.R12.T @ C11 + self.R23d3.T @ C22)
        self.J4d3[3:6, 0] = np.cross(self.R34.T @ self.R23d3.T @ self.R12.T @ w101,
                                     self.R34.T @ self.R23.T @ self.R12.T @ C11 + self.R34.T @ self.R23.T @ C22 + self.R34.T @ C33 + B4) + np.cross(
            self.R34.T @ self.R23.T @ self.R12.T @ w101,
            self.R34.T @ self.R23d3.T @ self.R12.T @ C11 + self.R34.T @ self.R23d3.T @ C22)
        self.J5d3[3:6, 0] = np.cross(self.R45.T @ self.R34.T @ self.R23d3.T @ self.R12.T @ w101,
                                     self.R45.T @ self.R34.T @ self.R23.T @ self.R12.T @ C11 + self.R45.T @ self.R34.T @ self.R23.T @ C22 + self.R45.T @ self.R34.T @ C33 + self.R45.T @ C44 + B5) + np.cross(
            self.R45.T @ self.R34.T @ self.R23.T @ self.R12.T @ w101,
            self.R45.T @ self.R34.T @ self.R23d3.T @ self.R12.T @ C11 + self.R45.T @ self.R34.T @ self.R23d3.T @ C22)
        self.J6d3[3:6, 0] = np.cross(self.R56.T @ self.R45.T @ self.R34.T @ self.R23d3.T @ self.R12.T @ w101,
                                     self.R56.T @ self.R45.T @ self.R34.T @ self.R23.T @ self.R12.T @ C11 + self.R56.T @ self.R45.T @ self.R34.T @ self.R23.T @ C22 + self.R56.T @ self.R45.T @ self.R34.T @ C33 + self.R56.T @ self.R45.T @ C44 + self.R56.T @ C55 + B6) + np.cross(
            self.R56.T @ self.R45.T @ self.R34.T @ self.R23.T @ self.R12.T @ w101,
            self.R56.T @ self.R45.T @ self.R34.T @ self.R23d3.T @ self.R12.T @ C11 + self.R56.T @ self.R45.T @ self.R34.T @ self.R23d3.T @ C22)
        self.J3d3[3:6, 1] = np.cross(self.R23d3.T @ w212, self.R23.T @ C22 + B3) + np.cross(self.R23.T @ w212,
                                                                                            self.R23d3.T @ C22)
        self.J4d3[3:6, 1] = np.cross(self.R34.T @ self.R23d3.T @ w212,
                                     self.R34.T @ self.R23.T @ C22 + self.R34.T @ C33 + B4) + np.cross(
            self.R34.T @ self.R23.T @ w212, self.R34.T @ self.R23d3.T @ C22)
        self.J5d3[3:6, 1] = np.cross(self.R45.T @ self.R34.T @ self.R23d3.T @ w212,
                                     self.R45.T @ self.R34.T @ self.R23.T @ C22 + self.R45.T @ self.R34.T @ C33 + self.R45.T @ C44 + B5) + np.cross(
            self.R45.T @ self.R34.T @ self.R23.T @ w212, self.R45.T @ self.R34.T @ self.R23d3.T @ C22)
        self.J6d3[3:6, 1] = np.cross(self.R56.T @ self.R45.T @ self.R34.T @ self.R23d3.T @ w212,
                                     self.R56.T @ self.R45.T @ self.R34.T @ self.R23.T @ C22 + self.R56.T @ self.R45.T @ self.R34.T @ C33 + self.R56.T @ self.R45.T @ C44 + self.R56.T @ C55 + B6) + np.cross(
            self.R56.T @ self.R45.T @ self.R34.T @ self.R23.T @ w212,
            self.R56.T @ self.R45.T @ self.R34.T @ self.R23d3.T @ C22)

        self.J4d4 = np.zeros((6,6))
        self.J5d4 = np.zeros((6,6))
        self.J6d4 = np.zeros((6,6))

        self.J4d4[0:3, 0] = self.R34d4.T @ self.R23.T @ self.R12.T @ w101
        self.J5d4[0:3, 0] = self.R45.T @ self.R34d4.T @ self.R23.T @ self.R12.T @ w101
        self.J6d4[0:3, 0] = self.R56.T @ self.R45.T @ self.R34d4.T @ self.R23.T @ self.R12.T @ w101
        self.J4d4[0:3, 1] = self.R34d4.T @ self.R23.T @ w212
        self.J5d4[0:3, 1] = self.R45.T @ self.R34d4.T @ self.R23.T @ w212
        self.J6d4[0:3, 1] = self.R56.T @ self.R45.T @ self.R34d4.T @ self.R23.T @ w212
        self.J4d4[0:3, 2] = self.R34d4.T @ w323
        self.J5d4[0:3, 2] = self.R45.T @ self.R34d4.T @ w323
        self.J6d4[0:3, 2] = self.R56.T @ self.R45.T @ self.R34d4.T @ w323
        self.J4d4[3:6, 0] = np.cross(self.R34d4.T @ self.R23.T @ self.R12.T @ w101,
                                     self.R34.T @ self.R23.T @ self.R12.T @ C11 + self.R34.T @ self.R23.T @ C22 + self.R34.T @ C33 + B4) + np.cross(
            self.R34.T @ self.R23.T @ self.R12.T @ w101,
            self.R34d4.T @ self.R23.T @ self.R12.T @ C11 + self.R34d4.T @ self.R23.T @ C22 + self.R34d4.T @ C33)
        self.J5d4[3:6, 0] = np.cross(self.R45.T @ self.R34d4.T @ self.R23.T @ self.R12.T @ w101,
                                     self.R45.T @ self.R34.T @ self.R23.T @ self.R12.T @ C11 + self.R45.T @ self.R34.T @ self.R23.T @ C22 + self.R45.T @ self.R34.T @ C33 + self.R45.T @ C44 + B5) + np.cross(
            self.R45.T @ self.R34.T @ self.R23.T @ self.R12.T @ w101,
            self.R45.T @ self.R34d4.T @ self.R23.T @ self.R12.T @ C11 + self.R45.T @ self.R34d4.T @ self.R23.T @ C22 + self.R45.T @ self.R34d4.T @ C33)
        self.J6d4[3:6, 0] = np.cross(self.R56.T @ self.R45.T @ self.R34d4.T @ self.R23.T @ self.R12.T @ w101,
                                     self.R56.T @ self.R45.T @ self.R34.T @ self.R23.T @ self.R12.T @ C11 + self.R56.T @ self.R45.T @ self.R34.T @ self.R23.T @ C22 + self.R56.T @ self.R45.T @ self.R34.T @ C33 + self.R56.T @ self.R45.T @ C44 + self.R56.T @ C55 + B6) + np.cross(
            self.R56.T @ self.R45.T @ self.R34.T @ self.R23.T @ self.R12.T @ w101,
            self.R56.T @ self.R45.T @ self.R34d4.T @ self.R23.T @ self.R12.T @ C11 + self.R56.T @ self.R45.T @ self.R34d4.T @ self.R23.T @ C22 + self.R56.T @ self.R45.T @ self.R34d4.T @ C33)
        self.J4d4[3:6, 1] = np.cross(self.R34.T @ self.R23.T @ w212,
                                     self.R34d4.T @ self.R23.T @ C22 + self.R34.T @ C33 + B4) + np.cross(
            self.R34.T @ self.R23.T @ w212, self.R34d4.T @ self.R23.T @ C22 + self.R34d4.T @ C33)
        self.J5d4[3:6, 1] = np.cross(self.R45.T @ self.R34d4.T @ self.R23.T @ w212,
                                     self.R45.T @ self.R34.T @ self.R23.T @ C22 + self.R45.T @ self.R34.T @ C33 + self.R45.T @ C44 + B5) + np.cross(
            self.R45.T @ self.R34.T @ self.R23.T @ w212,
            self.R45.T @ self.R34d4.T @ self.R23.T @ C22 + self.R45.T @ self.R34d4.T @ C33)
        self.J6d4[3:6, 1] = np.cross(self.R56.T @ self.R45.T @ self.R34d4.T @ self.R23.T @ w212,
                                     self.R56.T @ self.R45.T @ self.R34.T @ self.R23.T @ C22 + self.R56.T @ self.R45.T @ self.R34.T @ C33 + self.R56.T @ self.R45.T @ C44 + self.R56.T @ C55 + B6) + np.cross(
            self.R56.T @ self.R45.T @ self.R34.T @ self.R23.T @ w212,
            self.R56.T @ self.R45.T @ self.R34d4.T @ self.R23.T @ C22 + self.R56.T @ self.R45.T @ self.R34d4.T @ C33)
        self.J4d4[3:6, 2] = np.cross(self.R34.T @ w323, self.R34d4.T @ C33 + B4) + np.cross(self.R34.T @ w323,
                                                                                            self.R34d4.T @ C33)
        self.J5d4[3:6, 2] = np.cross(self.R45.T @ self.R34d4.T @ w323,
                                     self.R45.T @ self.R34.T @ C33 + self.R45.T @ C44 + B5) + np.cross(
            self.R45.T @ self.R34.T @ w323, self.R45.T @ self.R34d4.T @ C33)
        self.J6d4[3:6, 2] = np.cross(self.R56.T @ self.R45.T @ self.R34d4.T @ w323,
                                     self.R56.T @ self.R45.T @ self.R34.T @ C33 + self.R56.T @ self.R45.T @ C44 + self.R56.T @ C55 + B6) + np.cross(
            self.R56.T @ self.R45.T @ self.R34.T @ w323, self.R56.T @ self.R45.T @ self.R34d4.T @ C33)
        self.J5d5 = np.zeros((6,6))
        self.J6d5 = np.zeros((6,6))
        self.J5d5[0:3, 0] = self.R45d5.T @ self.R34.T @ self.R23.T @ self.R12.T @ w101
        self.J6d5[0:3, 0] = self.R56.T @ self.R45d5.T @ self.R34.T @ self.R23.T @ self.R12.T @ w101
        self.J5d5[0:3, 1] = self.R45d5.T @ self.R34.T @ self.R23.T @ w212
        self.J6d5[0:3, 1] = self.R56.T @ self.R45d5.T @ self.R34.T @ self.R23.T @ w212
        self.J5d5[0:3, 2] = self.R45d5.T @ self.R34.T @ w323
        self.J6d5[0:3, 2] = self.R56.T @ self.R45d5.T @ self.R34.T @ w323
        self.J5d5[0:3, 3] = self.R45d5.T @ w434
        self.J6d5[0:3, 3] = self.R56.T @ self.R45d5.T @ w434
        self.J5d5[3:6, 0] = np.cross(self.R45d5.T @ self.R34.T @ self.R23.T @ self.R12.T @ w101,
                                     self.R45.T @ self.R34.T @ self.R23.T @ self.R12.T @ C11 + self.R45.T @ self.R34.T @ self.R23.T @ C22 + self.R45.T @ self.R34.T @ C33 + self.R45.T @ C44 + B5) + np.cross(
            self.R45.T @ self.R34.T @ self.R23.T @ self.R12.T @ w101,
            self.R45d5.T @ self.R34.T @ self.R23.T @ self.R12.T @ C11 + self.R45d5.T @ self.R34.T @ self.R23.T @ C22 + self.R45d5.T @ self.R34.T @ C33 + self.R45d5.T @ C44)
        self.J6d5[3:6, 0] = np.cross(self.R56.T @ self.R45d5.T @ self.R34.T @ self.R23.T @ self.R12.T @ w101,
                                     self.R56.T @ self.R45.T @ self.R34.T @ self.R23.T @ self.R12.T @ C11 + self.R56.T @ self.R45.T @ self.R34.T @ self.R23.T @ C22 + self.R56.T @ self.R45.T @ self.R34.T @ C33 + self.R56.T @ self.R45.T @ C44 + self.R56.T @ C55 + B6) + np.cross(
            self.R56.T @ self.R45.T @ self.R34.T @ self.R23.T @ self.R12.T @ w101,
            self.R56.T @ self.R45d5.T @ self.R34.T @ self.R23.T @ self.R12.T @ C11 + self.R56.T @ self.R45d5.T @ self.R34.T @ self.R23.T @ C22 + self.R56.T @ self.R45d5.T @ self.R34.T @ C33 + self.R56.T @ self.R45d5.T @ C44)
        self.J5d5[3:6, 1] = np.cross(self.R45d5.T @ self.R34.T @ self.R23.T @ w212,
                                     self.R45.T @ self.R34.T @ self.R23.T @ C22 + self.R45.T @ self.R34.T @ C33 + self.R45.T @ C44 + B5) + np.cross(
            self.R45.T @ self.R34.T @ self.R23.T @ w212,
            self.R45d5.T @ self.R34.T @ self.R23.T @ C22 + self.R45d5.T @ self.R34.T @ C33 + self.R45d5.T @ C44)
        self.J6d5[3:6, 1] = np.cross(self.R56.T @ self.R45d5.T @ self.R34.T @ self.R23.T @ w212,
                                     self.R56.T @ self.R45.T @ self.R34.T @ self.R23.T @ C22 + self.R56.T @ self.R45.T @ self.R34.T @ C33 + self.R56.T @ self.R45.T @ C44 + self.R56.T @ C55 + B6) + np.cross(
            self.R56.T @ self.R45.T @ self.R34.T @ self.R23.T @ w212,
            self.R56.T @ self.R45d5.T @ self.R34.T @ self.R23.T @ C22 + self.R56.T @ self.R45d5.T @ self.R34.T @ C33 + self.R56.T @ self.R45d5.T @ C44)
        self.J5d5[3:6, 2] = np.cross(self.R45d5.T @ self.R34.T @ w323,
                                     self.R45.T @ self.R34.T @ C33 + self.R45.T @ C44 + B5) + np.cross(
            self.R45.T @ self.R34.T @ w323, self.R45d5.T @ self.R34.T @ C33 + self.R45d5.T @ C44)
        self.J6d5[3:6, 2] = np.cross(self.R56.T @ self.R45d5.T @ self.R34.T @ w323,
                                     self.R56.T @ self.R45.T @ self.R34.T @ C33 + self.R56.T @ self.R45.T @ C44 + self.R56.T @ C55 + B6) + np.cross(
            self.R56.T @ self.R45.T @ self.R34.T @ w323,
            self.R56.T @ self.R45d5.T @ self.R34.T @ C33 + self.R56.T @ self.R45d5.T @ C44)
        self.J5d5[3:6, 3] = np.cross(self.R45d5.T @ w434, self.R45.T @ C44 + B5) + np.cross(self.R45.T @ w434,
                                                                                            self.R45d5.T @ C44)
        self.J6d5[3:6, 3] = np.cross(self.R56.T @ self.R45d5.T @ w434,
                                     self.R56.T @ self.R45.T @ C44 + self.R56.T @ C55 + B6) + np.cross(
            self.R56.T @ self.R45.T @ w434, self.R56.T @ self.R45d5.T @ C44)
        self.J6d6 = np.zeros((6,6))
        self.J6d6[0:3, 0] = self.R56d6.T @ self.R45.T @ self.R34.T @ self.R23.T @ self.R12.T @ w101
        self.J6d6[0:3, 1] = self.R56d6.T @ self.R45.T @ self.R34.T @ self.R23.T @ w212
        self.J6d6[0:3, 2] = self.R56d6.T @ self.R45.T @ self.R34.T @ w323
        self.J6d6[0:3, 3] = self.R56d6.T @ self.R45.T @ w434
        self.J6d6[0:3, 4] = self.R56d6.T @ w545
        self.J6d6[3:6, 0] = np.cross(self.R56d6.T @ self.R45.T @ self.R34.T @ self.R23.T @ self.R12.T @ w101,
                                     self.R56.T @ self.R45.T @ self.R34.T @ self.R23.T @ self.R12.T @ C11 + self.R56.T @ self.R45.T @ self.R34.T @ self.R23.T @ C22 + self.R56.T @ self.R45.T @ self.R34.T @ C33 + self.R56.T @ self.R45.T @ C44 + self.R56.T @ C55 + B6) + np.cross(
            self.R56d6.T @ self.R45.T @ self.R34.T @ self.R23.T @ self.R12.T @ w101,
            self.R56d6.T @ self.R45.T @ self.R34.T @ self.R23.T @ self.R12.T @ C11 + self.R56d6.T @ self.R45.T @ self.R34.T @ self.R23.T @ C22 + self.R56d6.T @ self.R45.T @ self.R34.T @ C33 + self.R56d6.T @ self.R45.T @ C44 + self.R56d6.T @ C55)
        self.J6d6[3:6, 1] = np.cross(self.R56d6.T @ self.R45.T @ self.R34.T @ self.R23.T @ w212,
                                     self.R56.T @ self.R45.T @ self.R34.T @ self.R23.T @ C22 + self.R56.T @ self.R45.T @ self.R34.T @ C33 + self.R56.T @ self.R45.T @ C44 + self.R56.T @ C55 + B6) + np.cross(
            self.R56.T @ self.R45.T @ self.R34.T @ self.R23.T @ w212,
            self.R56d6.T @ self.R45.T @ self.R34.T @ self.R23.T @ C22 + self.R56d6.T @ self.R45.T @ self.R34.T @ C33 + self.R56d6.T @ self.R45.T @ C44 + self.R56d6.T @ C55)
        self.J6d6[3:6, 2] = np.cross(self.R56d6.T @ self.R45.T @ self.R34.T @ w323,
                                     self.R56.T @ self.R45.T @ self.R34.T @ C33 + self.R56.T @ self.R45.T @ C44 + self.R56.T @ C55 + B6) + np.cross(
            self.R56.T @ self.R45.T @ self.R34.T @ w323,
            self.R56d6.T @ self.R45.T @ self.R34.T @ C33 + self.R56d6.T @ self.R45.T @ C44 + self.R56d6.T @ C55)
        self.J6d6[3:6, 3] = np.cross(self.R56d6.T @ self.R45.T @ w434,
                                     self.R56.T @ self.R45.T @ C44 + self.R56.T @ C55 + B6) + np.cross(
            self.R56.T @ self.R45.T @ w434, self.R56d6.T @ self.R45.T @ C44 + self.R56d6.T @ C55)
        self.J6d6[3:6, 4] = np.cross(self.R56d6.T @ w545, self.R56.T @ C55 + B6) + np.cross(self.R56.T @ w545,
                                                                                            self.R56d6.T @ C55)
        Md = np.zeros([6,6,6])
        Md[0,:,:] = np.zeros((6,6))
        Md[1,:,:]=self.J2d2.T @ I2 @ self.J2 + self.J2.T @ I2 @ self.J2d2 + self.J3d2.T @ I3 @ self.J3 + self.J3.T @ I3 @ self.J3d2 + self.J4d2.T @ I4 @ self.J4 + self.J4.T @ I4 @ self.J4d2 + self.J5d2.T @ I5 @ self.J5 + self.J5.T @ I5 @ self.J5d2 + self.J6d2.T @ I6 @ self.J6 + self.J6.T @ I6 @ self.J6d2
        Md[2,:,:]=self.J3d3.T @ I3 @ self.J3 + self.J3.T @ I3 @ self.J3d3 + self.J4d3.T @ I4 @ self.J4 + self.J4.T @ I4 @ self.J4d3 + self.J5d3.T @ I5 @ self.J5 + self.J5.T @ I5 @ self.J5d3 + self.J6d3.T @ I6 @ self.J6 + self.J6.T @ I6 @ self.J6d3
        Md[3,:,:]=self.J4d4.T @ I4 @ self.J4 + self.J4.T @ I4 @ self.J4d4 + self.J5d4.T @ I5 @ self.J5 + self.J5.T @ I5 @ self.J5d4 + self.J6d4.T @ I6 @ self.J6 + self.J6.T @ I6 @ self.J6d4
        Md[4,:,:]=self.J5d5.T @ I5 @ self.J5 + self.J5.T @ I5 @ self.J5d5 + self.J6d5.T @ I6 @ self.J6 + self.J6.T @ I6 @ self.J6d5
        Md[5,:,:]=self.J6d6.T @ I6 @ self.J6 + self.J6.T @ I6 @ self.J6d6

        return calc_coriolis(Md)

    def precomputed_torque(self, q, qp, qpp):


        couple = self.inertia(q) @ qpp.T + self.coriolis(q,qp) @ qp.T + self.gravity(q)
        # print(max(self.inertia(q) @ qpp.T), max(self.coriolis(q,qp) @ qp.T), max(self.gravity(q)))
        return couple


if __name__ == "__main__":

    path = 'trajectory.csv'
    dyn = Dynamics()
    M = np.genfromtxt(path,delimiter=',')
    N = M.shape[0]
    couple = np.zeros((N,6))

    for i in range(N):
        q = M[i,1:7]
        qp = M[i,7:13]
        qpp = M[i,13:19]
        couple[i,0:6] = dyn.precomputed_torque(q,qp,qpp).T
        # print(i,couple[i,0:6])

    import matplotlib.pyplot as plt

    plt.figure(1)
    plt.subplot(611)
    plt.plot(M[:, 0], couple[:, 0])
    plt.title('Torque vs time')
    plt.ylabel('J_1 (Nm)')
    plt.subplot(612)
    plt.plot(M[:, 0], couple[:, 1])
    plt.ylabel('J_2 (Nm)')
    plt.subplot(613)
    plt.plot(M[:, 0], couple[:, 2])
    plt.ylabel('J_3 (Nm)')
    plt.subplot(614)
    plt.plot(M[:, 0], couple[:, 3])
    plt.ylabel('J_4 (Nm)')
    plt.subplot(615)
    plt.plot(M[:, 0], couple[:, 4])
    plt.ylabel('J_5 (Nm)')
    plt.subplot(616)
    plt.plot(M[:, 0], couple[:, 5])
    plt.ylabel('J_6 (Nm)')
    plt.xlabel('Time(s)')

    plt.show()