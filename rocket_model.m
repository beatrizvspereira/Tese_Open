% Constants
l1=10; % longitudinal length between the COG of the rocket and Fe
l2=3; % longitudinal length between the COG of the rocket and Fr,Fl
ln=5; % lentgh of the nozzle
m=25.4; % sum of the rocket's dry mass and fuel mass;
g=9.8; %gravity
lower_body=l1+ln;
upper_body=l2;

% Inputs

% tau=5;
% Tf = 7;
% Ts= 0.01;
% [u0,t] = gensig("square",tau,Tf, Ts);
% Fe = 6000*u0-1;
%phi=[(0)*ones(1,2000), (-5)*ones(1,1000),(5)*ones(1,5000)]';

%t=0:0.01:0.5; f=1; Fe=7.6*10^6*sin(2*pi*f*t); 
% Fe=Fe';
% t=t';

Fe=ones(1000,1)*(300); % Main thruster force
%Fe=6000;
Fr=ones(1000,1)*0; % Right thruster force
Fl=ones(1000,1)*0; % Left thruster force
Fs=Fl-Fr; 
%phi=0;
phi=ones(1000,1)*(0); % Angle between the x axis of the plan and the longitudinal axis of the rocket
theta=0; % Angle between the z axis of the plan and the longitudinal axis of the rocket

x=0; % horizontal position
z=0; % vertical position

J_T=10; % moment of inertia

% dd_xc= (Fe*theta+Fe*phi+Fs)/m;
% dd_z= (Fe-Fe*phi*theta-Fs*theta-m*g)/m;
% dd_theta=(-Fe*phi*(l1+ln)+Fs*l2)/J_T;

%% Nonlinear System Simulation - Altitude
%t=linspace(1,200,1000);
%t=t';
out=sim('alt_nonlinear',200);

% States
% x_1=out.yout{1};
% x_2=out.yout{2};
% x_3=out.yout{3};
% x_4=out.yout{4};
% x_5=out.yout{5};
% x_6=out.yout{6};

x_1=out.x_c;
x_2=out.dot_x_c;
x_3=out.z;
x_4=out.dot_z;
x_5=out.theta;
x_6=out.dot_theta;
tout=out.tout;
thrust=out.thrust;

% Input
figure;
plot(thrust);
title('Thrust signal');

% Altitude 
figure;
plot(x_3);
title('Vertical Position of the Rocket');
legend('z');

% Position
figure;
plot(x_1);
title('Horizontal Position of the Rocket');
legend('x');

% Velocity
figure;
plot(x_2);
title('Horizontal Velocity of the Rocket');
legend('dxdt');

% Velocity
figure;
plot(x_4);
title('Vertical Velocity of the Rocket');
legend('dzdt');

% Torque
figure;
plot(x_5);
title('Torque');
legend('theta');

% Velocity
figure;
plot(x_2);
title('Angular Velocity of the Rocket');
legend('dtheta/dt');

plot(x_1.data,x_3.data)
xlabel('x') 
ylabel('z') 
title('rocket 2D trajectory');

%% Nonlinear System Simulation - Gimbal
t=linspace(1,200,8000);
t=t';
out=sim('gimbal_nonlinear');

% States
x_1=out.x_c;
x_2=out.dot_x_c;
x_3=out.z;
x_4=out.dot_z;
x_5=out.theta;
x_6=out.dot_theta;
tout=out.time;
phi=out.phi;

% Input
figure;
plot(phi);
title('Gimbal angle');

% Altitude 
figure;
plot(x_3);
title('Vertical Position of the Rocket');
legend('z');

% Position
figure;
plot(x_1);
title('Horizontal Position of the Rocket');
legend('x');

% Torque
figure;
plot(x_5);
title('Torque');
legend('theta');

% Velocity
figure;
plot(x_2);
title('Horizontal Velocity of the Rocket');
legend('dx/dt');

% Velocity
figure;
plot(x_4);
title('Vertical Velocity of the Rocket');
legend('dz/dt');

% Velocity
figure;
plot(x_6);
title('Angular Velocity of the Rocket');
legend('dtheta/dt');

plot(x_1.data,x_3.data)
xlabel('x') 
ylabel('z') 
title('rocket 2D trajectory');
%%

A=[0,1,0,0,0,0;
   0,0,0,0,Fe/m,0;
   0,0,0,1,0,0;
   0,0,0,0,(-Fe*phi-Fs)/m,0;
   0,0,0,0,0,1;
   0,0,0,0,0,0];

B=[0,0,0;
   (theta+phi)/m, 1/m, Fe/m;
   0,0,0;
   (1-theta*phi)/m, -theta/m, (-Fe*theta)/m;
   0,0,0;
   (-phi*(l1+ln))/J_T, l2/J_T, (-Fe*(l1+ln))/J_T];

C=eye(6);
D=0;

x0=[0 0 0 0 0 0]'; % Initial Condition

sys= ss(A,B,C,D);

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
R=0.1; %Weight for the input variable

K=lqr(sys,Q,R); % Regulator gain K

% Estimator Design (LQE)

[Lc,Cc]=size(C);
G = eye(size(A))*(1e-4); %Gain of the process noise
Qe = eye(size(A))*10; %Variance of process errors
Re = eye(Lc)*(1e-4); %Variance of measurement errors

L = lqe(A,G,C,Qe,Re); % Estimator gain L (Kalman filter)

Aobs=A-L*C;
Bobs=[B L];
Cobs=eye(6);
Dobs=0*Bobs;

% t = (-1:0.01:1)';
% unitstep = t>=0;
% u=unitstep;

% Simulation 

T=30; % Simulation time
out=sim('rocket_sim',T);

tout=out.t;
yout=out.y;
uout=out.u;

figure;
plot(tout,yout);
hl=legend('$x$','$\dot{x}$','$z$', '$\dot{z}$','$\theta$', '$\dot{\theta}$','Location','northwest');
set(hl, 'Interpreter', 'latex');
set(hl,'Fontsize',10);


