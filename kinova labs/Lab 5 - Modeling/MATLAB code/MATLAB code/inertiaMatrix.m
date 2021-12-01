function [M] = inertiaMatrix(q)
m1=0.9597;
m2=1.1776;
m3=0.59768;
m4=0.52693;
m5=0.58097;
m6=0.2018;
w101=[0;0;1];
w212=[0;0;1];
w323=[0;0;1];
w434=[0;0;1];
w545=[0;0;1];
w656=[0;0;1];
B1=[0.000025;0.022135;0.099377];
B2=[0.029983;0.21155;0.045303];
B3=[0.030156;0.095022;0.007356];
B4=[0.005752;0.010004;0.087192];
B5=[0.080565;0.009804;0.018728];
B6=[0.00993;0.00995;0.06136];
C11=[0;-0.03;0.115];
C22=[0;-0.28;0];
C33=[0;-0.14;0.02];
C44=[0.0285;0;0.105];
C55=[-0.105;0;0.0285];
R12=[cosd(q(2)) -sind(q(2)) 0
    0 0 -1
    sind(q(2)) cosd(q(2)) 0];
R23=[cosd(q(3)) -sind(q(3)) 0 
    -sind(q(3)) -cosd(q(3)) 0 
    0 0 -1];
R34=[cosd(q(4)) -sind(q(4)) 0
    0 0 -1
    sind(q(4)) cosd(q(4)) 0];
R45=[0 0 1
    sind(q(5)) cosd(q(5)) 0
    -cosd(q(5)) sind(q(5)) 0];
R56=[0 0 -1
    sind(q(6)) cosd(q(6)) 0
    cosd(q(6)) -sind(q(6)) 0];
J1=zeros(6);
J2=zeros(6);
J3=zeros(6);
J4=zeros(6);
J5=zeros(6);
J6=zeros(6);
J1(1:3,1)=w101;
J2(1:3,1)=R12'*w101;
J3(1:3,1)=R23'*R12'*w101;
J4(1:3,1)=R34'*R23'*R12'*w101;
J5(1:3,1)=R45'*R34'*R23'*R12'*w101;
J6(1:3,1)=R56'*R45'*R34'*R23'*R12'*w101;
J2(1:3,2)=w212;
J3(1:3,2)=R23'*w212;
J4(1:3,2)=R34'*R23'*w212;
J5(1:3,2)=R45'*R34'*R23'*w212;
J6(1:3,2)=R56'*R45'*R34'*R23'*w212;
J3(1:3,3)=w323;
J4(1:3,3)=R34'*w323;
J5(1:3,3)=R45'*R34'*w323;
J6(1:3,3)=R56'*R45'*R34'*w323;
J4(1:3,4)=w434;
J5(1:3,4)=R45'*w434;
J6(1:3,4)=R56'*R45'*w434;
J5(1:3,5)=w545;
J6(1:3,5)=R56'*w545;
J6(1:3,6)=w656;
J1(4:6,1)=cross(w101,B1);
J2(4:6,1)=cross(R12'*w101,R12'*C11+B2);
J3(4:6,1)=cross(R23'*R12'*w101,R23'*R12'*C11+R23'*C22+B3);
J4(4:6,1)=cross(R34'*R23'*R12'*w101,R34'*R23'*R12'*C11+R34'*R23'*C22+R34'*C33+B4);
J5(4:6,1)=cross(R45'*R34'*R23'*R12'*w101,R45'*R34'*R23'*R12'*C11+R45'*R34'*R23'*C22+R45'*R34'*C33+R45'*C44+B5);
J6(4:6,1)=cross(R56'*R45'*R34'*R23'*R12'*w101,R56'*R45'*R34'*R23'*R12'*C11+R56'*R45'*R34'*R23'*C22+R56'*R45'*R34'*C33+R56'*R45'*C44+R56'*C55+B6);
J2(4:6,2)=cross(w212,B2);
J3(4:6,2)=cross(R23'*w212,R23'*C22+B3);
J4(4:6,2)=cross(R34'*R23'*w212,R34'*R23'*C22+R34'*C33+B4);
J5(4:6,2)=cross(R45'*R34'*R23'*w212,R45'*R34'*R23'*C22+R45'*R34'*C33+R45'*C44+B5);
J6(4:6,2)=cross(R56'*R45'*R34'*R23'*w212,R56'*R45'*R34'*R23'*C22+R56'*R45'*R34'*C33+R56'*R45'*C44+R56'*C55+B6);
J3(4:6,3)=cross(w323,B3);
J4(4:6,3)=cross(R34'*w323,R34'*C33+B4);
J5(4:6,3)=cross(R45'*R34'*w323,R45'*R34'*C33+R45'*C44+B5);
J6(4:6,3)=cross(R56'*R45'*R34'*w323,R56'*R45'*R34'*C33+R56'*R45'*C44+R56'*C55+B6);
J4(4:6,4)=cross(w434,B4);
J5(4:6,4)=cross(R45'*w434,R45'*C44+B5);
J6(4:6,4)=cross(R56'*R45'*w434,R56'*R45'*C44+R56'*C55+B6);
J5(4:6,5)=cross(w545,B5);
J6(4:6,5)=cross(R56'*w545,R56'*C55+B6);
J6(4:6,6)=cross(w656,B6);
I1=[0.0016595 0.00000002 0.00000036 0 0 0
    0.00000002 0.0014036 0.00034927 0 0 0
    0.00000036 0.00034927 0.00089493 0 0 0
    0 0 0 m1 0 0
    0 0 0 0 m1 0
    0 0 0 0 0 m1];
I2=[0.011493 0.000001 0.00000016 0 0 0
    0.000001 0.0010285 0.0014077 0 0 0
    0.00000016 0.0014077 0.011335 0 0 0
    0 0 0 m2 0 0
    0 0 0 0 m2 0
    0 0 0 0 0 m2];
I3=[0.0016326 0.0000071 0.0000015 0 0 0
    0.0000071 0.000298 0.000096 0 0 0
    0.0000015 0.000096 0.0016909 0 0 0
    0 0 0 m3 0 0
    0 0 0 0 m3 0
    0 0 0 0 0 m3];
I4=[0.00069098 0.00000024 0.00016483 0 0 0
    0.00000024 0.00078519 0.00000074 0 0 0
    0.00016483 0.00000074 0.00034115 0 0 0
    0 0 0 m4 0 0
    0 0 0 0 m4 0
    0 0 0 0 0 m4];
I5=[0.00021268 0.00000521 0.00000291 0 0 0
    0.00000521 0.0010637 0.00000011 0 0 0
    0.00000291 0.00000011 0.0010847 0 0 0
    0 0 0 m5 0 0
    0 0 0 0 m5 0
    0 0 0 0 0 m5];
I6=[0.0003428 0.00000019 0.0000001 0 0 0
    0.00000019 0.00028915 0.00000027 0 0 0
    0.0000001 0.00000027 0.00013076 0 0 0
    0 0 0 m6 0 0
    0 0 0 0 m6 0
    0 0 0 0 0 m6];
M=J1'*I1*J1+J2'*I2*J2+J3'*I3*J3+J4'*I4*J4+J5'*I5*J5+J6'*I6*J6;
end