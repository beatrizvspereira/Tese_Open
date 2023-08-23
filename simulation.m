function [thrust, gimbal, x_1,x_2,x_3,x_4,x_5,x_6,d_theta,d2_theta,d_x,d2_x,d_z,d2_z,ts] =simulation(T_stop)

%nonlinear_sys;
out=sim('nonlinear_simp_sys',T_stop); % Simplified Nonlinear model with Fs=0

ts=out.ts; %vector of sampling instants

% Inputs
thrust=out.thrust;
gimbal=out.gimbal;

% states
x_1=out.x;
x_2=out.dot_x;
x_3=out.z;
x_4=out.dot_z;
x_5=out.theta;
x_6=out.dot_theta;

d_theta=out.d_theta;
d2_theta=out.d2_theta;

d_x=out.d_x;
d2_x=out.d2_x;

d_z=out.d_z;
d2_z=out.d2_z;

% Plots

% Input
figure;
plot(ts,thrust,'LineWidth',1.6);
xlabel('t[s]','FontSize', 14); 
ylabel('FE [N]','FontSize', 14); 
title('Main Thrust signal','Fontsize', 14);

figure;
plot(ts,gimbal,'LineWidth',1.6);
xlabel('t[s]','FontSize', 14); 
ylabel('gimbal angle [rad]','FontSize', 14); 
title('Gimbal angle','Fontsize', 14);

% Altitude z
figure;
plot(ts,x_3,'LineWidth',1.6);
ylim([-100, 4100]);
xlabel('t[s]','FontSize', 14); 
ylabel('z [m]','FontSize', 14); 
yline(0,'--','LineWidth',1.6)
title('Vertical Position of the Rocket','FontSize', 14);
%legend('z','FontSize', 14);

% Position x
figure;
plot(ts,x_1,'LineWidth',1.6);
title('Horizontal Position of the Rocket','FontSize', 14);
xlabel('t[s]','FontSize', 14); 
ylabel('x [m]','FontSize', 14); 
%legend('x');

% Velocity x
figure;
plot(ts,x_2,'LineWidth',1.6);
title('Horizontal Velocity of the Rocket', 'Fontsize', 14);
xlabel('t[s]','FontSize', 14); 
ylabel('dx/dt [m/s]','FontSize', 14); 
%legend('dxdt');

% Velocity z
figure;
plot(ts,x_4,'LineWidth',1.6);
title('Vertical Velocity of the Rocket', 'Fontsize', 14);
xlabel('t[s]','FontSize', 14); 
ylabel('dz/dt [m/s]','FontSize', 14); 
%legend('dzdt');

% Theta
figure;
plot(ts,x_5,'LineWidth',1.6);
title('Angle \theta', 'Interpreter', 'tex', 'Fontsize', 14);
xlabel('t[s]','FontSize', 14); 
ylabel('\theta [rad]','Interpreter', 'tex','Fontsize', 14); 

% Velocity
figure;
plot(ts,x_2,'LineWidth',1.6);
title('Angular Velocity of the Rocket','FontSize', 14);
xlabel('t[s]','FontSize', 14); 
lx= ylabel('$\dot{\theta}$ [rad/s]'); 
set(lx, 'Interpreter', 'latex');
set(lx,'Fontsize',14);
%legend('dtheta/dt');

% 2D
figure;
plot(x_1,x_3,'LineWidth',1.6)
xlabel('x [m]','FontSize', 14); 
ylabel('z [m]', 'FontSize', 14); 
title('2D trajectory', 'FontSize', 14);

% 3D
figure;
plot3(x_1,zeros(size(x_1)),x_3,'LineWidth',1.6)
xlabel('x [m]','FontSize', 14); 
zlabel('z [m]', 'FontSize', 14); 
title('3D trajectory','FontSize', 14);


end