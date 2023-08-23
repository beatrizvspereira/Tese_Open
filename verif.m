
out=sim('tf_rocketmodel.slx',60);
out_z=sim('verif_rocketmodel',30);

toutput=out.ts;
theta_verif=out.theta;
lintheta=out.theta_linear;
%input_disturb=out.input_dist;

z_verif=out_z.z;

linz=out_z.z_lin;
toutput_z=out_z.ts;

figure;
plot(toutput,theta_verif,'LineWidth',1.8);
hold on;
plot(toutput,lintheta,'--','LineWidth',1.8);
%hold on;
%plot(toutput,input_disturb);
title('Linearized System VS Nonlinear System','Fontsize',14);
xlabel('t[s]','FontSize', 14);
ylabel('Angle [rad]','FontSize', 14);
legend('theta','linear theta');
hold off

figure;
plot(toutput_z,z_verif,'LineWidth',1.8);
hold on;
plot(toutput_z,linz,'m--','LineWidth',1.8);
%hold on;
%plot(toutput,input_disturb);
title('Linearized System VS Nonlinear System','Fontsize',14);
xlabel('t[s]','FontSize', 14);
ylabel('z [m]','FontSize', 14);
legend('z','linear z');
hold off

%% Função de Transferência do Controlador

A=[0,1;0,0];
B=[0; 1];
C=[1,0];
D=0;

K=[1, sqrt(2)];
L=[5;25];

sys=ss(A,B,C,D);

lqg_ss=tf(-reg(sys,K,L));
