function [K,L,K_lqr_z,K_z,sys,H]= rocket_controllerdesign_z(A,B,C,D)

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
Q=C'*C; %Weight Matrix for x
R=10; %Weight for the input variable

B=B*C(2);
C=[0,1];
sys=ss(A,B,C,D);

% optimal gain matrix K 
K = lqr(A,B,Q,R);
poles = eig(A-B*K)
Nbar=rscale(A,B,C,D,K);

Ac1=A-B*K;
sys_cl=ss(Ac1,B*Nbar,C,D);
K_lqr_z=tf(sys_cl);

figure;
step(sys_cl);
legend('Q=CC & R=10')

% Observer Design
[Lc,Cc]=size(C);
G = eye(size(A))*(1e-4); %Gain of the process noise
Qe = eye(size(A))*500; %Variance of process errors
Re = eye(Lc)*(1e-5); %Variance of measurement errors

L = lqe(A,G,C,Qe,Re);
eig_observer= eig(A-B*K-L*C)

lqg_ss=-reg(sys,K,L); % negative Feedback
K_z=tf(lqg_ss);

H=feedback(sys,K_z); %Closed-loop (Negative Feedback)

figure;
step(lqg_ss);
legend('RE=1e-5');
title('LQG: Closed-loop Step Response')

end