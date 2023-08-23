function [x_goal,z_goal,dz_goal] = nominal_trajectory(t,T_stop,t_signal,t_on)
theta_goal=0;

%gimbal angle
phi=zeros(length(t_signal),1); 

assignin('base','t_on',t_on);
output=sim('nonlinearsys_nominal', T_stop);

x_goal=output.x;
z_goal=output.z;
dz_goal=output.dot_z;

% Nominal z
figure;
plot(t,z_goal)
xlabel('z [m]','FontSize', 14); 
ylabel('t [s]', 'FontSize', 14); 
title('Nominal Verical Position', 'FontSize', 14);

% Nominal dz
figure;
plot(t,dz_goal)
xlabel('z [m]','FontSize', 14); 
ylabel('t [s]', 'FontSize', 14); 
title('Nominal Verical Velocity', 'FontSize', 14);


end