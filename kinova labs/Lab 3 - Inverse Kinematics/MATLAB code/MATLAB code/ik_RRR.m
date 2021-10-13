function [theta1,theta2,theta3] = ik_RRR(Wdes,soltype)
%q=[theta1 theta2 theta3]
%Params
l1=0.2433;
d1=0.010;
l2=0.280;
l3=0.245;
d=sqrt(Wdes(1)^2+Wdes(2)^2+(Wdes(3)-l1)^2-d1^2);
alpha=atan2d(Wdes(3)-l1,real(sqrt(Wdes(1)^2+Wdes(2)^2-d1^2)));
if soltype(1)=='r'
    theta1=atan2d(Wdes(2),Wdes(1))+asind(d1/sqrt(Wdes(2)^2+Wdes(1)^2));
    if soltype(2)=='d'
        theta2=-90+alpha-acosd((l2^2+d^2-l3^2)/(2*d*l2));
        theta3=-acosd((d^2-l2^2-l3^2)/(2*l2*l3));
    else
        theta2=-90+alpha+acosd((l2^2+d^2-l3^2)/(2*d*l2));
        theta3=acosd((d^2-l2^2-l3^2)/(2*l2*l3));
    end
else
    theta1=atan2d(Wdes(2),Wdes(1))-asind(d1/sqrt(Wdes(2)^2+Wdes(1)^2))+180;
    if soltype(2)=='u'
        theta2=90-alpha-acosd((l2^2+d^2-l3^2)/(2*d*l2));
        theta3=-acosd((d^2-l2^2-l3^2)/(2*l2*l3));
    else
        theta2=-270-alpha+acosd((l2^2+d^2-l3^2)/(2*d*l2));
        theta3=acosd((d^2-l2^2-l3^2)/(2*l2*l3));
    end
end
theta1=wrapTo180(real(theta1));
theta2=wrapTo180(real(theta2));
theta3=wrapTo180(real(theta3));
end

