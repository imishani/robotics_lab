function [q] = ik_gen3_lite_iterative(T,theta6_init,soltype,solnumber)
dwe=0.235;
dwo=0.0285*2;   
theta6k1=theta6_init;
theta6k=3*180;
i=1;
imax=200;
epsilon=1e-3;
while (i<imax)&&(abs(theta6k-theta6k1)>epsilon)
    W=T(1:3,4)+dwo*(sind(theta6k1)*T(1:3,2)-cosd(theta6k1)*T(1:3,1))-dwe*T(1:3,3);
    [theta1,theta2,theta3] = ik_RRR(W,soltype);
    R1=rotation([0;0;0],[0 0 1],real(theta1));
    R2=rotation([0 ; 0 ; 0.2433],[0 1 0],-real(theta2));
    R3=rotation([0 ; 0 ; 0.5233],[0 1 0],real(theta3));
    R=(R1(1:3,1:3)*R2(1:3,1:3)*R3(1:3,1:3))'*T(1:3,1:3)*[0 1 0;-1 0 0;0 0 1];
    [theta4,theta5,theta6]=R2ZXZ(R,solnumber);
    q=[real(theta1);real(theta2);real(theta3);theta4;theta5;theta6];
    theta6k=theta6k1;
    theta6k1=theta6;
    i=i+1;
end
if i==imax
    q=[NaN,NaN,NaN,NaN,NaN,NaN];
end
end

