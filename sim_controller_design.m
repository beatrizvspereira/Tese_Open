function s0_r0=sim_controller_design(P)

t = (-1:0.01:1)';
unitstep = t>=0;
inp=unitstep;

% State-Space Representation
[num,den] = tfdata(P);
[A,B,C,D] = tf2ss(num{1},den{1});
sys = ss(A,B,C,D);

% Controlability
Co =  ctrb(A,B);
if rank(Co) ~= rank(A)
    disp('Not Controlable');
end

% Observability
Ob = obsv(A,C);
if rank(Ob) ~= rank(A)
     disp('Not observable');
end

% Regulator Design (LQR)

Q=C'*C;
R=1e-4;

% optimal gain matrix K 
K = lqr(A,B,Q,R);

Nbar= -inv(C*inv(A-B*K)*B);

% Estimator Design (LQE)

[Lc,~]=size(C);
G = eye(size(A)); %Gain of the process noise
Qe = eye(size(A))*100; %Variance of process errors
Re = eye(size(Lc))*50; %Variance of measurement errors

L = lqe(A,G,C,Qe,Re);

Aobs=A-L*C;
Bobs=[B L];
Cobs=eye(2);
Dobs=0*Bobs;

out= sim('control_sys');

tout=out.t;
yout=out.y;
plot(tout,yout);

[a,b,c,d]=linmod('control_sys');
ctrl=ss(a,b,c,d);
[num,den] = tfdata(ctrl);
%[num,den] = ss2tf(a,b,c,d);
s0_r0 = tf(num{1},den{1}); % Controller 

end