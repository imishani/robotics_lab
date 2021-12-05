function [couple] = gravity(q)
Tb1=[1 0 0 0.000025
    0 1 0 0.022135
    0 0 1 0.099377
    0 0 0 1];
Tb2=[1 0 0 0.029983
    0 1 0 0.21155
    0 0 1 0.045303
    0 0 0 1];
Tb3=[1 0 0 0.030156
    0 1 0 0.095022
    0 0 1 0.007356
    0 0 0 1];
Tb4=[1 0 0 0.005752
    0 1 0 0.010004
    0 0 1 0.087192
    0 0 0 1];
Tb5=[1 0 0 0.080565
    0 1 0 0.009804
    0 0 1 0.018728
    0 0 0 1];
Tb6=[1 0 0 0.00993
    0 1 0 0.00995
    0 0 1 0.06136
    0 0 0 1];
T01=[cosd(q(1)) -sind(q(1)) 0 0
    sind(q(1)) cosd(q(1)) 0 0
    0 0 1 0.1283
    0 0 0 1];
T12=[cosd(q(2)) -sind(q(2)) 0 0
    0 0 -1 -0.03
    sind(q(2)) cosd(q(2)) 0 0.115
    0 0 0 1];
T23=[cosd(q(3)) -sind(q(3)) 0 0
    -sind(q(3)) -cosd(q(3)) 0 0.28
    0 0 -1 0
    0 0 0 1];
T34=[cosd(q(4)) -sind(q(4)) 0 0
    0 0 -1 -0.14
    sind(q(4)) cosd(q(4)) 0 0.02
    0 0 0 1];
T45=[0 0 -1 0.0285
    sind(q(5)) cosd(q(5)) 0 0
    cosd(q(5)) -sind(q(5)) 0 0.105
    0 0 0 1];
T56=[0 0 -1 -0.105
    sind(q(6)) cosd(q(6)) 0 0
    cosd(q(6)) -sind(q(6)) 0 0.0285
    0 0 0 1];
T01d1=[-sind(q(1)) -cosd(q(1)) 0 0
    cosd(q(1)) -sind(q(1)) 0 0
    0 0 0 0
    0 0 0 0];
T12d2=[-sind(q(2)) -cosd(q(2)) 0 0
    0 0 0 0
    cosd(q(2)) -sind(q(2)) 0 0
    0 0 0 0];
T23d3=[-sind(q(3)) -cosd(q(3)) 0 0
    -cosd(q(3)) sind(q(3)) 0 0
    0 0 0 0
    0 0 0 0];
T34d4=[-sind(q(4)) -cosd(q(4)) 0 0
    0 0 0 0
    cosd(q(4)) -sind(q(4)) 0 0
    0 0 0 0];
T45d5=[0 0 0 0
    cosd(q(5)) -sind(q(5)) 0 0
    -sind(q(5)) -cosd(q(5)) 0 0
    0 0 0 0];
T56d6=[0 0 0 0
    cosd(q(6)) -sind(q(6)) 0 0
    -sind(q(6)) -cosd(q(6)) 0 0
    0 0 0 0];
m1=0.9597;
m2=1.1776;
m3=0.59768;
m4=0.52693;
m5=0.58097;
m6=0.2018;
g0=m1*T01d1*Tb1+m2*T01d1*T12*Tb2+m3*T01d1*T12*T23*Tb3+m4*T01d1*T12*T23*T34*Tb4+m5*T01d1*T12*T23*T34*T45*Tb5+m6*T01d1*T12*T23*T34*T45*T56*Tb6;
g1=m2*T01*T12d2*Tb2+m3*T01*T12d2*T23*Tb3+m4*T01*T12d2*T23*T34*Tb4+m5*T01*T12d2*T23*T34*T45*Tb5+m6*T01*T12d2*T23*T34*T45*T56*Tb6;
g2=m3*T01*T12*T23d3*Tb3+m4*T01*T12*T23d3*T34*Tb4+m5*T01*T12*T23d3*T34*T45*Tb5+m6*T01*T12*T23d3*T34*T45*T56*Tb6;
g3=m4*T01*T12*T23*T34d4*Tb4+m5*T01*T12*T23*T34d4*T45*Tb5+m6*T01*T12*T23*T34d4*T45*T56*Tb6;
g4=m5*T01*T12*T23*T34*T45d5*Tb5+m6*T01*T12*T23*T34*T45d5*T56*Tb6;
g5=m6*T01*T12*T23*T45*T56d6*Tb6;
couple=9.81*[g0(3,4) g1(3,4) g2(3,4) g3(3,4) g4(3,4) g5(3,4)];
end
