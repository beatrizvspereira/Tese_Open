function [K,L,K_theta,K_lqr,sys]= rocket_controllerdesign(A,B,C,D,P)

%load('ss_model');

% Controlability
Co =  ctrb(A,B);
if rank(Co) ~= size(A)
    disp('Not Controlable');
end

% Observability
Ob = obsv(A,C);
if rank(Ob) ~= size(A)
     disp('Not observable');
end

% Regulator Design 
B= B*C(2);
C=[0,1];
sys = ss(A,B,C,D);

% optimal gain matrix K 
Q=C'*C; %Weight Matrix for x
R=1e-6; %Weight for the input variable
%R=1;  

%R_l=1; %Weight for the input variable

K = lqr(A,B,Q,R);
%K_l = lqr(A,B,Q,R_l);
Nbar=rscale(A,B,C,D,K);

poles = eig(A-B*K)
%poles_l = eig(A-B*K_l)

Ac1=A-B*K;
%Ac_l=A-B*K_l;

sys_cl=ss(Ac1,B*Nbar,C,D);
%sys_cl_l=ss(Ac_l,B,C,D);

K_lqr=tf(sys_cl);
%K_lqr_l=tf(sys_cl_l);

% [b,a]=ss2tf(sys_cl.A,sys_cl.B,sys_cl.C,sys_cl.D);
% K_lqr=tf(b,a); % LQR controller Transfer Function

H_1=feedback(sys,K_lqr); % Negative Feedback
%H_l=feedback(sys,K_lqr_l); % Negative Feedback

figure;
step(sys_cl);
% hold on;
% step(sys_cl_l);
% hold off
%legend('Q=CC & R=1e-6', 'Q=CC & R=1')
legend('Q=CC & R=1e-6');
title('LQr: Closed-loop Step Response');


% figure;
% step(sys_cl);

% Observer Design
[Lc,Cc]=size(C);
G = eye(size(A)); %Gain of the process noise
Qe = [0.1, 0;
       0,  1e-6]; %Variance of process errors
Re = eye(Lc)*1e-6; %Variance of measurement errors
%Re_l = eye(Lc)*1; %Variance of measurement errors

L = lqe(A,G,C,Qe,Re);
%L_l = lqe(A,G,C,Qe,Re_l);

eig_observer= eig(A-B*K-L*C)

lqg_ss=-reg(sys,K,L) % negative Feedback
%lqg_ss_l=-reg(sys,K,L_l) % negative Feedback

K_theta=tf(lqg_ss);
%K_theta_l=tf(lqg_ss_l);

% sysKF= ss(A-L*C,[B L],eye(2),0*[B L]);  % Kalman filter estimator
% [num,den]=ss2tf(lqg_ss.A,lqg_ss.B,lqg_ss.C,lqg_ss.D);
% K_theta = tf(num,den); % LQG Controller Transfer Function

H=feedback(sys,K_theta); %Closed-loop (Negative Feedback)
%H_ll=feedback(sys,K_theta_l); %Closed-loop (Negative Feedback)

figure;
step(lqg_ss);
legend('RE=1e-6');
title('LQG: Closed-loop Step Response')
% hold on;
% step(lqg_ss_l);
% hold off

% figure;
% sigma(H);

% figure;
% rlocus(H);
% figure;
% rlocus(sys*K_theta);


end