function [phi,theta,psi] = R2ZXZ(R,solnumber)
if R(3,3)==1
    %eul_angles=[phi theta psi];
    eul_angles=[atan2d(R(2,1),R(2,2)) 0 0];
else
    if solnumber %negative theta
        eul_angles=[atan2d(-R(1,3),R(2,3)) atan2d(-sqrt(R(1,3)^2+R(2,3)^2),R(3,3)) atan2d(-R(3,1),-R(3,2))];
    else % positive theta
        eul_angles=[atan2d(R(1,3),-R(2,3)) atan2d(sqrt(R(1,3)^2+R(2,3)^2),R(3,3)) atan2d(R(3,1),R(3,2))];
    end
end
phi=wrapTo180(eul_angles(1));
theta=wrapTo180(eul_angles(2));
psi=wrapTo180(eul_angles(3));
end

