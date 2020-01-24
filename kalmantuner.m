%% import data
clear;clc;
DT=0.01;
data = importdata("quadlog.txt",",");
dt=data(:,1);phi=data(:,2);theta=data(:,3);gx=data(:,4);gy=data(:,5);power=data(:,6);
%% data stats
% At hovering
ss_start= 2000; ss_end= 4500;
phi_ss = detrend(phi(ss_start:ss_end));
theta_ss = detrend(theta(ss_start:ss_end));
phidot_ss = detrend(gx(ss_start:ss_end));
thetadot_ss = detrend(gy(ss_start:ss_end));
figure(1); 
% sgtitle('Cross-Correlation of Euler Angles in Steady State Hover');
subplot(1,6,1);plot(xcorr(phi_ss));subplot(1,6,2);plot(xcorr(theta_ss, phi_ss));subplot(1,6,3);plot(xcorr(theta_ss));
subplot(1,6,4);plot(xcorr(phidot_ss));subplot(1,6,5);plot(xcorr(thetadot_ss,phidot_ss));subplot(1,6,6);plot(xcorr(theta_ss));
figure(2); plot(abs(fft(phi_ss))); hold on; plot(abs(fft(theta_ss)));
title('Dominant 33 Hz in Euler FFT (Steady Hover)');
fprintf("Var(Phidot): %5.1f, Var(Phi): %5.1f, Var(Thetadot): %5.1f, Var(Theta): %5.1f\n",var(phidot_ss),var(phi_ss),var(thetadot_ss),var(theta_ss))
%% kalman estimator
F = eye(4); F(2,1)=DT; F(4,3)=DT;
G = zeros(4,4);
H = eye(4);
xhatrec = []; pkrec = [];
Qdist = eye(4);
Rnoise = 100*diag([150,950,450,1000]);
% Rnoise = diag([1,10000,1,20000]);
Pk_post = eye(1);
xhat_post = zeros(4,1);
for i = 1:size(phi)
    u=zeros(4,1);
    y = [gx(i),phi(i),gy(i),theta(i)];
%     y = [phi(i),theta(i)];
    [Kk,xhat_post,Pk_post]=kalman(F,G,H,y,Qdist,Rnoise,Pk_post,xhat_post,u);
    xhatrec = [xhatrec xhat_post];
    pkrec = [pkrec [Pk_post(1,1);Pk_post(2,2);Pk_post(3,3);Pk_post(4,4)]];
end
% figure(1);plot(gx);hold on;plot(xhatrec(1,:));
% figure(2);plot(gy);hold on;plot(xhatrec(3,:));
figure(3);plot(phi);hold on;plot(xhatrec(2,:),'g-');plot(zeros(1,size(phi,1)),'c--');
plot(5*ones(1,size(phi,1)),'c--');plot(-5*ones(1,size(phi,1)),'c--');
xlabel("Samples");ylabel("Phi")
figure(5);plot(theta);hold on;plot(xhatrec(4,:),'g-');plot(zeros(1,size(theta,1)),'c--');
plot(5*ones(1,size(phi,1)),'c--');plot(-5*ones(1,size(phi,1)),'c--');
xlabel("Samples");ylabel("Theta")
%% Kalman Error Estimate
figure(6);plot(xhatrec(2,:),'g-');plot(zeros(1,size(phi,1)),'c--')
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