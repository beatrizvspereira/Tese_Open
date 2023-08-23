function [s0_r0,Nbar,sys,K,E]=controller_design(P, flag_rocket)

% State-Space Representation
[num,den] = tfdata(P);
[A,B,C,D] = tf2ss(num{1},den{1});

if flag_rocket
B=B*C(2);
C=[0,1];
end

sys = ss(A,B,C,D);

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

% Regulator Design (LQR)

Q=C'*C; %Weight Matrix for x
R=1e-6; %Weight for the input variable
%R=1e-3;
%R=100;

% optimal gain matrix K 
[K,S,E] = lqr(A,B,Q,R);

Nbar=rscale(A,B,C,D,K);
%Nbar= -inv(C*inv(A-B*K)*B);
%BB = Nbar*B;
 
% sys_cl = ss((A-B*K),B,C,D);
% [b,a] = ss2tf(sys_cl.A,sys_cl.B,sys_cl.C,sys_cl.D);
% s0_r0 = tf(b,a); % Controller 

% Observer Design
[Lc,Cc]=size(C);
G = eye(size(A)); %Gain of the process noise
% Qe = eye(size(A))*1e3; %Variance of process errors
% Re = eye(Lc)*1e3; %Variance of measurement errors

%Qe = eye(size(A))*(1e-6); %Variance of process errors

Qe = [0.1, 0;
     0,  1e-6]; %Variance of process errors
Re = eye(Lc)*1e-6; %Variance of measurement errors

% Observer gain matrix L

% Pkal=ss(A,[B B],C,0);
% [est,L]=kalman(Pkal,inv(R),inv(Q));
% sys_e= ss(A,B,C,0);

L = lqe(A,G,C,Qe,Re);

%sys_cl_e= ss((A-B*K-L*C),L,-K,D);
% sys_cl_e= ss((A-L*C),[B L],eye(2),D);

sys_lqg=-reg(sys,K,L); % negative feedback
%sys_lqg=reg(sys,K,L); % positive feedback
s0_r0=tf(sys_lqg);

% [b,a] = ss2tf(sys_lqg.A,sys_lqg.B,sys_lqg.C,sys_lqg.D);
% s0_r0 = tf(b,a); % Controller 

H=feedback(sys,s0_r0); %Closed-loop

end