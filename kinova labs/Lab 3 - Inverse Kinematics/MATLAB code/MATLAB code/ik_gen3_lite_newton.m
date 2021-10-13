function [q] = cininv_gen3_lite_newton(T,q0)
epsilon=1e-2;
q=q0';
Vk=ones(6,1);
i=1;
imax=1000;
while (norm(Vk)>epsilon) && (i<imax)
    pose_k=fk_gen3_lite_dh(q);
    error=T/pose_k;
    R=error(1:3,1:3);
    if trace(R)==3
        theta=norm(T(1:3,4)-pose_k(1:3,4));
        omega=[0;0;0];
        v=(T(1:3,4)-pose_k(1:3,4))/norm(T(1:3,4)-pose_k(1:3,4));
        lambda=1;
    else
        if trace(R)==-1
            theta=90;
            omega1=sqrt((R(1,1)+1)/2);
            omega2=sign(R(1,2))*sqrt((R(2,2)+1)/2);
            omega3=sign(R(1,3))*sqrt((R(3,3)+1)/2);
            omega=[omega1;omega2;omega3];
        else
            theta=acosd((trace(R)-1)/2);
            omega=1/(2*sin(theta))*[R(3,2)-R(2,3);R(1,3)-R(3,1);R(2,1)-R(1,2)];
        end
        A=(eye(3)-R)*skewm(omega)+theta*(omega*omega');
        v=A\(T(1:3,4)-pose_k(1:3,4));
        lambda=1e-2;
    end
    Vk(1:3,1)=theta*omega;
    Vk(4:6,1)=theta*v;
    Js=jacob_gen3_lite_s(q);
    q=q+lambda*(Js'*Js)^(-1)*Js'*Vk;
    i=i+1;
end
if i==imax
    q=[NaN NaN NaN NaN NaN NaN];
end
    function [Te6] = fk_gen3_lite_dh(q)
        T56=transf_dh(0,0,0.105+0.130,q(6)+90);
        T45=transf_dh(0,90,0.0285*2,q(5)+180);
        T34=transf_dh(0,90,0.140+0.105,q(4)+90);
        T23=transf_dh(0,90,0.020,q(3)+90);
        T12=transf_dh(0.280,180,0.030,q(2)+90);
        T01=transf_dh(0,90,0.1283+0.115,q(1));
        Tes=[0 -1 0 0
        1 0 0 0
        0 0 1 0
        0 0 0 1];
        Te6=T01*T12*T23*T34*T45*T56*Tes;
        function [transf] = transf_dh(a,alpha,d,v)
            transf1=[cosd(v) -sind(v) 0 0
                sind(v) cosd(v) 0 0
                0 0 1 d
                0 0 0 1];
            transf2=[1 0 0 a
                0 cosd(alpha) -sind(alpha) 0
                0 sind(alpha) cosd(alpha) 0
                0 0 0 1];
            transf=transf1*transf2;
        end
    end
end

