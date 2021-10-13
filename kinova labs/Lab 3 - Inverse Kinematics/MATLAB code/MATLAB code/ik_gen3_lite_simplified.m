function [q] = ik_gen3_lite_simplified(T,soltype,solnumber)
dwe=0.235;
W=T(1:3,4)-dwe*T(1:3,3);
[theta1,theta2,theta3] = ik_RRR(W,soltype);
R1=rotation([0;0;0],[0 0 1],theta1);
R2=rotation([0 ; 0 ; 0.2433],[0 1 0],-theta2);
R3=rotation([0 ; 0 ; 0.5233],[0 1 0],theta3);
R=(R1(1:3,1:3)*R2(1:3,1:3)*R3(1:3,1:3))'*T(1:3,1:3)*[0 1 0;-1 0 0;0 0 1];
[theta4,theta5,theta6]=R2ZXZ(R,solnumber);
q=[theta1;theta2;theta3;theta4;theta5;theta6];
end

