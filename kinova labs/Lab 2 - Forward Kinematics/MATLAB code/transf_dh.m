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