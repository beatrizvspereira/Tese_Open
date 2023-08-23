g=9.8;
z_0=4000;
m= 549054;
z_on= zeros(1,6);
i=0;

for t_on=[12 13 14.55 14.6 14.75 14.8 14.9 15 19]
   i=i+1;
   z_on(i)=z_0+(1/2)*(-g+(0.05*7e6)/m)*(t_on^2);
%    v_on=-g*t_on;
%    tf=(-v_on+sqrt(v_on^2-4*(-10+7e6/m)*z_on))/2*(-10+7e6/m);
  
   clearvars ans sim
   sim('nonlinear_simp_sys', 60)
   
   sim=ans;
   t=sim.ts;
   z=sim.z;
   dz=sim.dot_z;
   
   figure(1)
   plot(t,z,'linewidth',1.6);
   ylim([-100, 4100]);
   hold on;
   
end

figure(1);
xlabel('t [s]','FontSize', 14); ylabel('z [m]','FontSize', 14);
yline(0,'--');
t_on=[12 13 14.55 14.6 14.75 14.8 14.9 15 19];
legend(sprintf('t_{ON}=%0.2f',t_on(1)),sprintf('t_{ON}=%0.2f',t_on(2)),sprintf('t_{ON}=%0.2f',t_on(3)),...
    sprintf('t_{ON}=%0.2f',t_on(4)),sprintf('t_{ON}=%0.2f',t_on(5)),sprintf('t_{0N}=%0.2f',t_on(6)),...
    sprintf('t_{ON}=%0.2f',t_on(7)),sprintf('t_{ON}=%0.2f',t_on(8)),sprintf('t_{ON}=%0.2f',t_on(9)),'z=0m',...
    'location','northeast','FontSize', 12);
title('Nominal Trajectory for different t_{ON} values','FontSize', 14);
hold off

%% plots

t_on= 14.6;
z_on=z_0+(1/2)*(-g+(0.05*7e6)/m)*(t_on^2);

clearvars ans sim
sim('nonlinear_simp_sys', 60)

sim=ans;
t=sim.ts;
z=sim.z;
dz=sim.dot_z;

figure;
plot(t,z);
ylim([-100, 4100]);
xlabel('t [s]','Fontsize',14); ylabel('z [m]','Fontsize',14);
yline(0,'--');
title('Vertical Position of the Rocket','Fontsize',14);

figure;
plot(t,dz);
xlabel('t [s]','Fontsize',14); ylabel('dz/dt [m/s]','Fontsize',14);
title('Vertical Velocity of the Rocket','Fontsize',14);