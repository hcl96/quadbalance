%% Quadcopter Estimation
clear;clc;
% Parameter Declaration
DT=0.01;
m=0.903;g=9.81;Ixx=0.356;Iyy=0.373;Izz=0.020;
arm_length=0.22;d=arm_length/sqrt(2);
R_prop=0.125; rho_air=1.225; coeff_thrust=0.1028*0.25;
m_motor=55.1;r_motor=0.015;
Ir=(1/2)*m_motor*r_motor^2;
timeconst = 0.5;
% Constants
K1=(d/Ixx)*sqrt(m*g*coeff_thrust*rho_air*pi*R_prop^4);
K2=(d/Iyy)*sqrt(m*g*coeff_thrust*rho_air*pi*R_prop^4);
% CT State Space Model
A=zeros(4,4); % x = [phidot,phi,thetadot,theta]
A(2,1)=1;A(4,3)=1;
B=zeros(4,4);
B(1,1:4)=K1*[1,-1,-1,1];
B(3,1:4)=K2*[1,1,-1,-1];
C=eye(4);
D=[];
sysC=ss(A,B,C,D); % CT state space
sysD=c2d(sysC,DT,'zoh'); % DT state space
rank(ctrb(A,B)) 
rank(obsv(A,C))
%% LQR Design w/ Bryson's Rule
Qerec=[deg2rad(20),deg2rad(1),deg2rad(20),deg2rad(1)]; %Max Tolerable Errors
Qc=diag([1/Qerec(1)^2,1/Qerec(2)^2,1/Qerec(3)^2,1/Qerec(4)^2]);
Rc=0.0001*diag([1,1,1,1]); % vary to change control response
Kc=lqr(A,B,Qc,Rc);
sysC_control=ss(A-B*Kc,[],C,[]);
eig(sysC.A-sysC.B*Kc)
Kd=dlqr(sysD.A,sysD.B,Qc,Rc);
sysD_control=ss(sysD.A-sysD.B*Kd,sysD.B,sysD.C,sysD.D,DT);
eig(sysD.A-sysD.B*Kd);
%% Kalman Filter Design
F=sysD.A;G=sysD.B;H=sysD.C;

xhatrec = [];
Qdist = eye(4);
Rnoise = diag([1,10000,1,10000]);
Pk_post = eye(1);
xhat_post = zeros(4,1);
for i = 1:size(phi)
    u=zeros(4,1);
    y = [gx(i),phi(i),gy(i),theta(i)];
%     y = [phi(i),theta(i)];
    [Kk,xhat_post,Pk_post]=kalman(F,G,H,y,Qdist,Rnoise,Pk_post,xhat_post,u);
    xhatrec = [xhatrec xhat_post];
end
% figure(1);plot(gx);hold on;plot(xhatrec(1,:));
% figure(2);plot(gy);hold on;plot(xhatrec(3,:));
figure(3);plot(phi);hold on;plot(xhatrec(2,:),'g-');plot(zeros(1,size(phi,1)),'c--')
plot(5*ones(1,size(phi,1)),'c--');plot(-5*ones(1,size(phi,1)),'c--');
xlabel("Samples");ylabel("Roll Angle")
figure(5);plot(theta);hold on;plot(xhatrec(4,:),'g-');plot(zeros(1,size(theta,1)),'c--')
plot(5*ones(1,size(phi,1)),'c--');plot(-5*ones(1,size(phi,1)),'c--');
xlabel("Samples");ylabel("Pitch Angle")
% plot(xhatrec(2,:)); plot(xhatrec(3,:)); plot(xhatrec(4,:))

%%
function [Kk,xhat_post,Pk_post]=kalman(F,G,H,y,Qdist,Rnoise,Pk_post,xhat_post,u)
   % time update
   Pk_pre = F*Pk_post*F'+Qdist;
   Kk = Pk_pre*H'*inv(H*Pk_pre*H'+Rnoise);
   xhat_pre = F*xhat_post+G*u;
   % measurement update
   xhat_post = xhat_pre + Kk*(y'-H*xhat_pre);
   Pk_post = (eye(4)-Kk*H)*Pk_pre*(eye(4)-Kk*H)'+Kk*Rnoise*Kk';
   % recursive update
   Pk_pre=Pk_post;
   xhat_pre=xhat_post;
end
