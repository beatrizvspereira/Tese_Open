clear;
clc;
close all;

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

% figure;
% plot(t_signal,phi);
% title('Gimbal Angle', 'Fontsize', 14);
% xlabel('t[s]','Fontsize', 14);
% ylabel('Input Gimbal Angle [rad]','Fontsize', 14);
% ylim([-1.2e-6,1.2e-6]);
% yline(0,'--');
% legend('Gimbal Angle', 'Equilibrium value','Fontsize', 14);

%% Gimbal= 0 rad and Thrust Force varying

%gimbal angle
phi=zeros(length(t_signal),1); 
%phi=zeros(1000,1);
%n=wgn(length(phi),1,-30); % white guassian noise with power -30dbW (1e-6 watt)

% Main thruster force
Fe=thrust_eq+(2e6)*square(t_signal,25);

% figure;
% plot(t_signal,Fe);
% title('Main Thrust Force', 'Fontsize', 14);
% xlabel('t[s]','Fontsize', 14);
% ylabel('Thrust [N]','Fontsize', 14);
% yline(m*g,'--');
% legend('Thurst Force', 'Equilibrium value','Fontsize', 14);

%% Simulation

[thrust, gimbal, x_1,x_2,x_3,x_4,x_5,x_6,d_theta,d2_theta,d_x,d2_x,d_z,d2_z,ts] = simulation(T_stop);

%% System Identification - Gimbal 
close all;

% Data for Identification
delta_angle= gimbal-gimbal_eq;

us2=delta_angle;
ys2=d2_theta;
save('id_data_atitude.mat', 'us2','ys2','ts');

% Identification
[A,B,C,D,alpha]=sys_identification_attitude();

%% System Identification - Thrust
close all;

% Data for Identification
delta_thrust= thrust-thrust_eq;

us3=delta_thrust;
ys3=d2_z;
save('id_data_altitude.mat', 'us3','ys3','ts');

% Identification
[A2,B2,C2,D2,beta]=sys_identification_altitude(); 

%% Controller Design - Attitude
close all;
P=tf(ss(A,B,C,D));
[K,L,K_theta,K_lqr,sys]=rocket_controllerdesign(A,B,C,D); % Outputs LQR gain matrix K for angle control
%Nbar=rscale(A,B,C,D,K);
disp(['Angle LQR K gain: ', num2str(K)]);
disp(['Observer Gains L: ', num2str(L')]);

%% Controller Design - Altitude
[K_2,L_2,K_lqr_z,K_z,sys2]=rocket_controllerdesign_z(A2,B2,C2,D2); % LQR gain matrix K for altitude control
Nbar_2=rscale(A2,B2,C2,D2,K_2);
disp(['Altitude LQR K2 gain: ', num2str(K_2)]);
disp(['Observer Gains L: ', num2str((L_2)')]);

%% Closed-Loop Simulation - TRANSFER FUNCTIONS (LQG)

close all;
s0_r0=K_theta;
theta_goal=0;

tic
%tf_rocketmodel
T_sim=60;
out1=sim('tf_rocketmodel',T_sim); % simulate rocket model

out_x=out1.x;
out_z=out1.z;
dz=out1.dot_z;
y_theta=out1.theta;
y_dot_theta=out1.dot_theta;
gimbal_signal=out1.gimbal;
thrust=out1.thrust;
tsout=out1.ts;
distub_gimbal=out1.disturb;
gimbal_control=out1.Phi_control;


% Tracking error mean square value
%error= sum((theta_goal - y_theta).^2); %sum of squared error
%disp(['theta SSE: ', num2str(error)]);
mse=sqrt(mean((theta_goal - y_theta).^2)); % root mean squared error
disp(['RMSE: ', num2str(mse)]);

% Tracking error mean square value
%error_trajectory= sum((z_goal - out_z).^2); %sum of squared error
%disp(['z SSE: ', num2str(error_trajectory)]);
mse_trajectory=sqrt(mean((z_goal - out_z).^2)); % root mean squared error
disp(['z RMSE: ', num2str(mse_trajectory)]);

% 2D controller trajectory
figure;
plot(out_x,out_z,'LineWidth', 1.6)
xlim([-1,1]);
xlabel('x [m]','FontSize', 14); 
ylabel('z [m]', 'FontSize', 14); 
title('Controlled 2D trajectory', 'FontSize', 14);

% % 2D controller trajectory
% figure;
% plot(out_x,out_z)
% xlim([-2,2]);
% xlabel('x [m]','FontSize', 14); 
% ylabel('z [m]', 'FontSize', 14); 
% title('Controlled 2D trajectory', 'FontSize', 14);

% 2D controller trajectory x
figure;
plot(tsout,out_x,'LineWidth', 1.6)
ylabel('x [m]','FontSize', 14); 
xlabel('t [s]', 'FontSize', 14); 
title('Horizontal Trajectory', 'FontSize', 14);

% 3D
figure;
plot3(out_x,zeros(size(out_x)),out_z,'LineWidth', 1.6)
xlim([-1,1]);
xlabel('x [m]','FontSize', 14); 
zlabel('z [m]', 'FontSize', 14); 
title('3D trajectory','FontSize', 14);

% Control Gimbal Sum
figure;
plot(tsout,distub_gimbal,'LineWidth', 1.6);
hold on;
plot(tsout,gimbal_control,'--','LineWidth', 1.6);
hold on;
plot(tsout,gimbal_signal,'LineWidth', 1.6);
xlabel('t[s]','FontSize', 14); 
ylabel('Angle [rad]','FontSize', 14); 
legend('Gimbal Angle Disturbance', 'Gimbal Controller Output', 'Sum of signals: Controlled Gimbal');
title('Sum of Input Signals','Fontsize', 14);
hold off

% Control Gimbal Sum (Degrees)
figure;
plot(tsout,rad2deg(distub_gimbal),'LineWidth', 1.6);
hold on;
plot(tsout,rad2deg(gimbal_control),'--','LineWidth', 1.6);
hold on;
plot(tsout,rad2deg(gimbal_signal),'LineWidth', 1.6);
xlabel('t[s]','FontSize', 14); 
ylabel('Angle [deg]','FontSize', 14); 
legend('Gimbal Angle Disturbance', 'Gimbal Controller Output', 'Sum of signals: Controlled Gimbal');
title('Sum of Input Signals','Fontsize', 14);
hold off

% Disturbance
figure;
plot(tsout,distub_gimbal,'LineWidth', 1.6);
xlabel('t[s]','FontSize', 14); 
ylabel('Angle [rad]','FontSize', 14); 
title('Gimbal Angle Disturbance','Fontsize', 14);

% Controlled Gimbal angle
figure;
plot(tsout,gimbal_signal,'LineWidth', 1.6);
% ylim([-0.2618, 0.2618]);
yline(-0.2618,'--');
yline(0.2618,'--');
xlim([-1,58]);
xlabel('t[s]','FontSize', 14); 
ylabel('Controlled Gimbal Angle [rad]','FontSize', 14); 
title('Gimbal angle','Fontsize', 14);

% Controlled Gimbal angle (Degrees)
figure;
plot(tsout,rad2deg(gimbal_signal),'LineWidth', 1.6);
xlabel('t[s]','FontSize', 14); 
ylabel('Controlled Gimbal Angle [deg]','FontSize', 14); 
title('Gimbal angle','Fontsize', 14);

% Controlled Thrust
figure;
plot(tsout,thrust,'LineWidth', 1.6);
xlabel('t[s]','FontSize', 14); 
%xlim([0,58]);
ylabel('Controlled Thrust Force [N]','FontSize', 14); 
title('Thrust Force','Fontsize', 14);

% Controlled Theta
figure;
plot(tsout,y_theta,'LineWidth', 1.6);
title('Angle \theta', 'Interpreter', 'tex', 'Fontsize', 14);
xlabel('t[s]','FontSize', 14); 
ylabel('\theta [rad]','Interpreter', 'tex','Fontsize', 14); 

% Controlled Theta (Degrees)
figure;
plot(tsout,rad2deg(y_theta),'LineWidth', 1.6);
title('Angle \theta', 'Interpreter', 'tex', 'Fontsize', 14);
xlabel('t[s]','FontSize', 14); 
ylabel('\theta [deg]','Interpreter', 'tex','Fontsize', 14); 

% Controlled dot theta (angular velocity)
figure;
plot(tsout,y_dot_theta,'LineWidth', 1.6);
title('Angular Velocity of the Rocket','FontSize', 14);
xlabel('t[s]','FontSize', 14); 
lx= ylabel('$\dot{\theta}$ [rad/s]'); 
set(lx, 'Interpreter', 'latex');
set(lx,'Fontsize',14);

% dot z (velocity)
figure;
plot(tsout,dz,'LineWidth', 1.6);
title('Vertical Velocity of the Rocket','FontSize', 14);
xlabel('t[s]','FontSize', 14); 
lx= ylabel('$\dot{z}$ [m/s]'); 
set(lx, 'Interpreter', 'latex');
set(lx,'Fontsize',14);

% vertical position 
figure;
plot(tsout,out_z,'LineWidth', 1.6);
title('Vertical Position','FontSize', 14);
xlabel('t[s]','FontSize', 14); 
ylabel('z [m]','FontSize', 14);

% Compare
figure;
plot(tsout,y_theta,'LineWidth', 1.6);
hold on;
plot(ts,x_5,'LineWidth', 1.6);
title('Compare Angle \theta', 'Interpreter', 'tex', 'Fontsize', 14);
legend('Closed-loop Angle', 'Open-loop Angle');
xlabel('t[s]','FontSize', 14); 
ylabel('\theta [rad]','Interpreter', 'tex','Fontsize', 14); 
hold off

% Compare
figure;
plot(tsout,z_goal,'LineWidth', 1.6);
hold on;
plot(tsout,out_z,'--','linewidth',1.6);
title('Vertical Position Comparison', 'Interpreter', 'tex', 'Fontsize', 14);
legend('nominal z','Closed-loop position z', 'Fontsize', 14);
xlabel('t[s]','FontSize', 14); 
ylabel('z [m]','Interpreter', 'tex','Fontsize', 14); 
hold off


toc

%% Closed-Loop Simulation - STATE SPACE (LQR)
close all
tic

rocketmodel
T_sim=60;
out1=sim('rocketmodel',T_sim); % simulate rocket model

x=out1.x;
z=out1.z;
der_z=out1.dot_z;
y_theta=out1.theta;
y_dot_theta=out1.dot_theta;
gimbal_signal=out1.gimbal;
thrust=out1.thrust;
tsout=out1.ts;
distub_gimbal=out1.disturb;
gimbal_control=out1.Phi_control;

% Tracking error mean square value
error= sum((theta - y_theta).^2); %sum of squared error
disp(['theta SSE: ', num2str(error)]);
mse=sqrt(mean((theta - y_theta).^2)); % root mean squared error
disp(['RMSE: ', num2str(mse)]);

% 2D controller trajectory
figure;
plot(x,z)
xlabel('x [m]','FontSize', 14); 
ylabel('z [m]', 'FontSize', 14); 
title('Controlled 2D trajectory', 'FontSize', 14);

% 3D
figure;
plot3(x,zeros(size(x)),z)
xlabel('x [m]','FontSize', 14); 
zlabel('z [m]', 'FontSize', 14); 
title('3D trajectory','FontSize', 14);

% Control Gimbal Sum
figure;
plot(tsout,distub_gimbal);
hold on;
plot(tsout,gimbal_control,'--');
hold on;
plot(tsout,gimbal_signal);
xlim([-1, 57]);
xlabel('t[s]','FontSize', 14); 
ylabel('Angle [rad]','FontSize', 14); 
legend('Gimbal Angle Disturbance', 'Gimbal Controller Output', 'Sum of signals: Controlled Gimbal');
title('Sum of Input Signals','Fontsize', 14);
hold off

% Disturbance
figure;
plot(tsout,distub_gimbal);
xlabel('t[s]','FontSize', 14); 
ylabel('Angle [rad]','FontSize', 14); 
title('Gimbal Angle Disturbance','Fontsize', 14);

% Controlled Gimbal angle
figure;
plot(tsout,gimbal_signal);
%ylim([-0.2618, 0.2618]);
xlim([-1,58]);
xlabel('t[s]','FontSize', 14); 
ylabel('Controlled Gimbal Angle [rad]','FontSize', 14); 
title('Gimbal angle','Fontsize', 14);

% Controlled Thrust
figure;
plot(tsout,thrust);
xlabel('t[s]','FontSize', 14); 
%xlim([0,58]);
ylabel('Controlled Thrust Force [N]','FontSize', 14); 
title('Thrust Force','Fontsize', 14);

% Controlled Theta
figure;
plot(tsout,y_theta);
title('Angle \theta', 'Interpreter', 'tex', 'Fontsize', 14);
%ylim([-0.2618,0.2618]);
xlim([0,58]);
xlabel('t[s]','FontSize', 14); 
ylabel('\theta [rad]','Interpreter', 'tex','Fontsize', 14); 

% Controlled dot theta (angular velocity)
figure;
plot(tsout,y_dot_theta);
title('Angular Velocity of the Rocket','FontSize', 14);
%ylim([-1e-4,1e-4]);
xlim([0,58]);
xlabel('t[s]','FontSize', 14); 
lx= ylabel('$\dot{\theta}$ [rad/s]'); 
set(lx, 'Interpreter', 'latex');
set(lx,'Fontsize',14);

% dot z (velocity)
figure;
plot(tsout,der_z);
title('Vertical Velocity of the Rocket','FontSize', 14);
xlabel('t[s]','FontSize', 14); 
lx= ylabel('$\dot{z}$ [m/s]'); 
set(lx, 'Interpreter', 'latex');
set(lx,'Fontsize',14);

figure;
plot(tsout,z);
title('Vertical Position','FontSize', 14);
xlabel('t[s]','FontSize', 14); 
ylabel('z [m]','FontSize', 14);

% Compare
figure;
plot(tsout,y_theta);
hold on;
plot(ts,x_5);
xlim([0, 57]);
title('Compare Angle \theta', 'Interpreter', 'tex', 'Fontsize', 14);
legend('Closed-loop Angle', 'Open-loop Angle');
xlabel('t[s]','FontSize', 14); 
ylabel('\theta [rad]','Interpreter', 'tex','Fontsize', 14); 
hold off

toc

%% Trajectory Comparisons

T_sim=60;
out1=sim('lqg_rocketmodel',T_sim); % simulate rocket model 
x=out1.x;
z=out1.z;

% LQR
out2=sim('rocketmodel',T_sim);
x_lqr=out2.x;
z_lqr=out2.z;

%tf
out3=sim('tf_rocketmodel',T_sim);
x_tf=out3.x;
z_tf=out3.z;

% 2D controller trajectory (nominal)
figure;
plot(x,z);
hold on
plot(x_tf,z_tf);
hold on;
plot(x_lqr,z_lqr);
%xlim([-0.02,0.02]);
xlabel('x [m]','FontSize', 14); 
ylabel('z [m]', 'FontSize', 14); 
title('2D trajectory Comparison', 'FontSize', 14);
legend('LQG Controlled Trajectory','LQG(tf) Controlled Trajectory','LQR Controlled Trajectory');
hold off 

% nominal 
figure;
plot(x_goal,z_goal);
hold on
plot(x_tf,z_tf);
hold on;
xlim([-1,1]);
xlabel('x [m]','FontSize', 14); 
ylabel('z [m]', 'FontSize', 14); 
title('2D trajectory Comparison', 'FontSize', 14);
legend('Nominal Trajectory','LQG Controlled Trajectory');
hold off 

%% linmod
% [a_l,b_l,c_l,d_l]=linmod('nonlinear_simp_sys');
% sys_linearized= ss(a_l,b_l,c_l,d_l);
