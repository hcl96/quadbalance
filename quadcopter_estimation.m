%% Quadcopter Estimation
clear;clc;
% Parameter Declaration
g=9.81;
Ixx=0.01366;Iyy=0.01337;Izz=0.02648;g=9.81;
arm_length=0.27;d=arm_length/sqrt(2);m=0.903;
R_prop=0.13; rho_air=1.225; coeff_thrust=0.01295;
m_motor=0.052;r_motor=0.015;
Ir=(1/2)*m_motor*r_motor^2;
% Continuous State Space Declaration: Model
K1=(d/Ixx)*sqrt(m*g*coeff_thrust*rho_air*pi*R_prop^4);
K2=(d/Iyy)*sqrt(m*g*coeff_thrust*rho_air*pi*R_prop^4);
A=zeros(4,4);A(2,1)=1;A(4,3)=1;
B=zeros(4,4);B(1,1:4)=K1*[1,-1,1,-1];B(3,1:4)=K2*[1,1,-1,-1];
%7000rpm at (1000base)+700 uSeconds: approx. 1 uS -> +10 rpm
% thrust at 7200 rpm = 7.553 N / + 950 rpm every +1N
%scaling factor for uSeconds -> motor
% should not scale here, keep models physical
C=eye(4);D=[];
sysC=ss(A,B,C,D);
rank(ctrb(A,B)); %full rank of 4 means system controllable
rank(obsv(A,C));
% lsim(sysC,10*rand(4,200),[0:0.05:10-0.05],rand(4,1)); %simulate 10s of random disturbances
% Discrete State Space Declaration
sysD=c2d(sysC,0.01,'zoh'); % assume clock speed of 10mS
%% LQR Design (CT)
Qc=eye(4);Qc(1,1)=0.1;Qc(3,3)=0.1;
Rc=0.00005*eye(4);
Kd=dlqr(sysD.A,sysD.B,Qc,Rc);
Kc=lqr(A,B,Qc,Rc);
sysC_control=ss(A-B*Kc,[],C,[]);
eig(sysC.A-sysC.B*Kc)
x0=[deg2rad(7),deg2rad(-8),deg2rad(5),deg2rad(-7)];
% initial(sysC_control,x0,0:0.1:10);
% LQR Design (DT)
sysD_control=ss(sysD.A-sysD.B*Kd,sysD.B,sysD.C,sysD.D,0.01);
eig(sysD.A-sysD.B*Kd)
[ysimd,tsimd,xsimd]=lsim(sysD_control,rand(4,1000),[0:0.01:10-0.01],0.1*rand(4,1));
plot(tsimd, ysimd(:,1),'-'); hold on;
plot(tsimd, ysimd(:,2),'--');
plot(tsimd, ysimd(:,3),'-');
plot(tsimd, ysimd(:,4),'--'); legend('phidot','phi','thetadot','theta')
Kd
%% Kalman Filter Design
Qdist=2*eye(4); Qdist(2,2)=0.65; Qdist(4,4)=0.2; % from var(phiraw)
Rnoise=10e-6*eye(4); Rnoise(1,1)=10e-5; Rnoise(3,3)=10e-5;
Rnoise=100000000*Rnoise;% optimal
F=sysD.A;G=sysD.B;H=sysD.C;
xhat=[];Kalman_gain=[];Pk=[];
xrec=[];Krec=[];Prec=[];
xhat0=zeros(4,1);
P0=0.001*eye(4);
Pk_post=P0;
xhat_post=xhat0;
%%
for n = 1:1000
    y(1,1:4) = ysimd(n,1:4);
    [Kk,xhat_pre,Pk_post]=kalman(F,G,H,y,Qdist,Rnoise,Pk_post,xhat_post,zeros(4,1));
    xrec=[xrec xhat_pre];
	Prec=[Prec trace(Pk_post)];
    Krec=[Krec sum(sum(Kk))];
end
plot(xrec(1,:));hold on;plot(ysimd(:,1));
% innovations=ysimd(1,:)'-H*xrec;
% plot(xcorr(detrend(innovations(1,:))))


%% Plot actual Data
% load imu_test_data.mat
subplot(2,2,[1 2])
plot(0:0.01:size(phiest)/100-0.01,phiest);hold on; plot(0:0.01:size(phiest)/100-0.01,phiraw)
title('Phi Raw vs Estimate'); xlabel('time (s)'); ylabel('degrees');
legend('phi_est','phi_raw');
subplot(2,2,[3 4])
plot(0:0.01:size(thetaest)/100-0.01,thetaest);hold on; plot(0:0.01:size(thetaest)/100-0.01,thetaraw)
title('Theta Raw vs Estimate'); xlabel('time (s)'); ylabel('degrees');
legend('theta_est','theta_raw');
hold off;
%%
function [Kk,xhat_post,Pk_post]=kalman(F,G,H,y,Qdist,Rnoise,Pk_post,xhat_post,u)
   % time update
   Pk_pre = F*Pk_post*F'+Qdist;
   Kk = Pk_pre*H'/(H*Pk_pre*H'+Rnoise);
   xhat_pre = F*xhat_post+G*u;
   % measurement update
   xhat_post = xhat_pre + Kk*(y'-H*xhat_pre);
   Pk_post = (eye(4)-Kk*H)*Pk_pre*(eye(4)-Kk*H)'+Kk*Rnoise*Kk';
   % recursive update
   Pk_pre=Pk_post;
   xhat_pre=xhat_post;
end



