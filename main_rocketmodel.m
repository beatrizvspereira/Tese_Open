% Main simulation of rocket model + baseline controllers
% Beatriz Pereira

%% State-Space Models

load('ss_model.mat') % Attitude System
load('ss_model_altitude.mat') % Altitude System

%% Inputs

% Constants
l1=60; % longitudinal length between the COG of the rocket and Fe
l2=10; % longitudinal length between the COG of the rocket and Fr,Fl
ln=1; % lentgh of the nozzle
m=549054; % sum of the rocket's dry mass and fuel mass;
g=9.8; %gravity

theta=0; % Angle between the z axis of the plan and the longitudinal axis of the rocket
J_T=40267; % moment of inertia

% Equilibrium Values
thrust_eq=m*g;
gimbal_eq=0;
Ts=0.001; % Sampling interval
T_stop=60;

% Signal
fs=1000;
t_signal=(0:1/fs:T_stop)';

%% Initial Conditions

% Phi 
phi=zeros(length(t_signal),1); 

% time
t=linspace(0,T_stop,length(phi));
t=t';

% Lateral thruster force
Fr=zeros(length(phi),1); % Right thruster force
Fl=zeros(length(phi),1); % Left thruster force
Fs=Fl-Fr; 

x_0=0; % horizontal position
z_0=4000; % vertical position

% Bang Bang Thrust 
t_on=14.6;
z_on=z_0+(1/2)*(-g+(0.05*7e6)/m)*(t_on^2);

%% Nominal Trajectory 
close all;
[x_goal,z_goal,dz_goal] = nominal_trajectory(t,T_stop,t_signal,t_on);

%% Gimbal varying with Constant Thrust Force

% Gimbal Angle
phi=(1e-6)*square(t_signal,25); % Angle between the x axis of the plan and the longitudinal axis of the rocket

% Main thruster force

%Fe=ones(length(phi),1)*(7.6*10^6); %ascend
Fe=ones(length(phi),1)*(m*g); %static

%% Simulation

[thrust, gimbal, x_1,x_2,x_3,x_4,x_5,x_6,d_theta,d2_theta,d_x,d2_x,d_z,d2_z,ts] = simulation(T_stop);

%% Controller Design - Attitude
close all;
P=tf(ss(A,B,C,D));
[K,L,K_theta,K_lqr,sys]=rocket_controllerdesign(A,B,C,D,P); % Outputs LQR gain matrix K for angle control
%Nbar=rscale(A,B,C,D,K);
disp(['Angle LQR K gain: ', num2str(K)]);
disp(['Observer Gains L: ', num2str(L')]);

%% Controller Design - Altitude
[K_2,L_2,K_lqr_z,K_z,sys2,H_z]=rocket_controllerdesign_z(A2,B2,C2,D2); % LQR gain matrix K for altitude control
Nbar_2=rscale(A2,B2,C2,D2,K_2);
disp(['Altitude LQR K2 gain: ', num2str(K_2)]);
disp(['Observer Gains L: ', num2str((L_2)')]);

%% Open-loop Bode Diagram
[NUM_theta, DEN_theta] = ss2tf(A,B,C,D);

figure;
bode(NUM_theta,DEN_theta);
