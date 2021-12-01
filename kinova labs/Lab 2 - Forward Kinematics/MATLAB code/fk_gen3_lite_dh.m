function [configs] = fk_gen3_lite_dh(q)
N=size(q);
configs=zeros(4,4,N(1));
Tes=[0 -1 0 0
    1 0 0 0
    0 0 1 0
    0 0 0 1];
for i=1:N(1)
T56=transf_dh(0,0,0.105+0.130,q(i,6)+90);
T45=transf_dh(0,90,0.0285*2,q(i,5)+180);
T34=transf_dh(0,90,0.140+0.105,q(i,4)+90);
T23=transf_dh(0,90,0.020,q(i,3)+90);
T12=transf_dh(0.280,180,0.030,q(i,2)+90);
T01=transf_dh(0,90,0.1283+0.115,q(i,1));
Te6=T01*T12*T23*T34*T45*T56*Tes;
configs(:,:,i)=Te6;
end