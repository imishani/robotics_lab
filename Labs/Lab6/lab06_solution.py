
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

def calc_coriolis(Md):
    cor = np.zeros((6, 6))
    for i in range(6):
        for j in range(6):
            for k in range(6):
                cor[i, j] = cor[i, j] + 0.5 * (Md[k, i, j] + Md[j, i, k] - Md[i, j, k]) * qp[k]

    return cor




