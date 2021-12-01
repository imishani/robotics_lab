M=csvread("trajectory.csv");
N=size(M);
couple=zeros(N(1),6);
for i=1:N(1)
    q=M(i,2:7);
    qp=M(i,8:13);
    qpp=M(i,14:19);
    couple(i,1:6)=precomputed_torque(q,qp,qpp)';
end
figure(1)
subplot(611)
plot(M(:,1),couple(:,1))
title('Torque vs time')
ylabel('J_1 (Nm)')
subplot(612)
plot(M(:,1),couple(:,2))
ylabel('J_2 (Nm)')
subplot(613)
plot(M(:,1),couple(:,3))
ylabel('J_3 (Nm)')
subplot(614)
plot(M(:,1),couple(:,4))
ylabel('J_4 (Nm)')
subplot(615)
plot(M(:,1),couple(:,5))
ylabel('J_5 (Nm)')
subplot(616)
plot(M(:,1),couple(:,6))
ylabel('J_6 (Nm)')
xlabel('Time(s)')
