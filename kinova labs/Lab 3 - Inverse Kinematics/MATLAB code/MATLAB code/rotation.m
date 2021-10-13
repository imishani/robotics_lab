function [transf] = rotation(Q,omega,theta)
wx=skewm(omega);
rotat=eye(3)+sind(theta)*wx+(1-cosd(theta))*wx^2;
transf=[rotat (eye(3)-rotat)*Q
    0 0 0 1];
end

