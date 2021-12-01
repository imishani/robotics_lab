function [configs] = fk_gen3_lite_rt(q)
N=size(q);
q=q*pi/180;
configs=zeros(4,4,N(1));
Qo_tool=[0.057;-0.010;1.0033];
rot_corr=[0 -1 0
    1 0 0
    0 0 1];
Tes=[rot_corr Qo_tool
    0 0 0 1];
Q6s=[0.057;-0.010;0.8733];
Q5s=[0.0285;-0.010;0.7683];
Q4s=[0;-0.010;0.6633];
Q3s=[0;-0.030;0.5233];
Q2s=[0;-0.030;0.2433];
Q1s=[0;0;0.1283];
for i=1:N(1)
T6s=rotation(Q6s,[0;0;1],q(i,6));
T5s=rotation(Q5s,[1;0;0],q(i,5));
T4s=rotation(Q4s,[0;0;1],q(i,4));
T3s=rotation(Q3s,[0;1;0],q(i,3));
T2s=rotation(Q2s,[0;-1;0],q(i,2));
T1s=rotation(Q1s,[0;0;1],q(i,1));
Te6=T1s*T2s*T3s*T4s*T5s*T6s*Tes;
configs(:,:,i)=Te6;
end



