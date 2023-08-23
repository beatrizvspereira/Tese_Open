%%  Main: Automatic design of Controller Parameters based on YP and Fminunc
% Beatriz Ventura dos Santos Pereira
% Nº90029

%% Input

% Signal
[u,t] = sum_squares_sig();

%[u,t]=square_sig();
%Ts_sig=0.03125;
Ts_sig=0.001;
%[u,t] = square_sig();

% System
flag_rocket=0;
[T,P, flag_rocket]=rocket_system();
%[T,P]=second_order();
%[T,P]=higher_order();

% Initial Vector Phi
phi_initial=-80;

% Alpha
alpha= 1;

% Noise seed
rng(100);

%% Initial LQG controller S0/R0

[s0_r0,Nbar,sys,K,E] = controller_design(P,flag_rocket);
figure;
step(s0_r0)
H_s0_r0=minreal((s0_r0*P)/(1+s0_r0*P));
CL_stability=isstable(H_s0_r0) %Negative Feedback

%% Fminunc (follow ref)
tic
close all;

N=4;

idx=1;
J_vec=zeros(1,N);

for idx = 1:N
close all;

% Open-loop Unstable
[phi_final, J, J_norm,output, K_controller_fminunc,H_fminunc,Q_fminunc] = fminunc_LQ(u,t, phi_initial,alpha,T,P,s0_r0);

Sh= isstable(H);
if Sh == 0
    disp('Closed-loop unstable');
end

disp('Fminunc:');
disp(output);
disp(['Final vector of Phi: ', num2str(phi_final),]);
disp(['Final cost: ', num2str(J), ' , Normalized cost: ', num2str(J_norm)]);

if N<2
    break
end

% Augment a dimension 
%close all;
phi_initial=[phi_final, 0];
disp(['Dimension:', num2str(length(phi_initial)),', Phi initial: ', num2str(phi_initial),]);

% Learning Progress
J_vec(idx)=J;
disp(['Learning: ', num2str(J_vec),]);

figure;
plot(J_vec);
set(gca,'Yscale','log');
xlabel('Iteration');
ylabel('Cost');
title('Learning Curve','Fontsize',14);

end
toc

% Metrics

T_sim=60;
[u,t] = sum_squares_sig();
ym= lsim(T,u,t);
yc= lsim(H_s0_r0,u,t);
yf=lsim(H_fminunc,u,t);
%y_con=lsim(H_con,u,t);
%yreinforce=lsim(H{k_min},u,t);

J_lqg = sum((yc - ym).^2); % Sum of squared error
J_yp=sum((yf - ym).^2);
%J_con=sum((y_con - ym).^2);
%J_reinforce=sum((yreinforce - ym).^2);

ratio_opti=(1-(J_yp/J_lqg))*100;
ratio_opti_2=(1-(J_reinforce/J_lqg))*100;


disp('Follow Reference Metrics: ')
disp(['cost LQG: ', num2str(J_lqg),]);
disp(['cost YK Controller: ', num2str(J_yp),]);
disp(['cost REINFORCE Controller: ', num2str(J_reinforce),]);
disp(['Optimization ratio fminunc (%): ', num2str(ratio_opti), '%']);
disp(['Optimization ratio REINFORCE (%): ', num2str(ratio_opti_2), '%']);

if J_yp<J_lqg
   disp('Controller was optimized to follow reference :)')
else 
   disp('Controller was NOT optimized to follow ref :(')
end

% System Response
figure;
plot(t, ym,'linewidth',1.6);
hold on;
plot(t, yf,'linewidth',1.8);
hold on;
plot(t, yc,'--','linewidth',1.8);
title('Follow Reference System with Fminunc','Fontsize',14);
legend('Reference','Response of the Fminunc Controller','Response of the LQG Controller','fontsize',14);
xlabel('t(ms)');
ylabel('y(t)');
hold off

% % REINFORCE 
% figure;
% plot(t, ym,'linewidth',1.6);
% hold on;
% plot(t, yreinforce,'--','linewidth',1.6);
% hold on;
% plot(t, yc,'-','linewidth',1);
% title('Follow Reference System','Fontsize',14);
% legend('Response of Reference Transfer Function','Response of the Episodic REINFORCE Controller','Response of the LQG Controller','fontsize',14);
% xlabel('t(ms)','Fontsize',14);
% ylabel('y(t)','Fontsize',14);
% hold off

% % REINFORCE vs fminunc
% figure;
% plot(t, yreinforce,'linewidth',1.5);
% hold on;
% plot(t, ym,'LineWidth',1.8);
% hold on
% plot(t, yf,'--','linewidth',1.6);
% title('Learning Algorithm VS Fminunc','Fontsize',14);
% legend('Response of the Episodic REINFORCE Controller','Response of the Reference System', 'Response of the Fminunc Controller','fontsize',14);
% xlabel('t(ms)','Fontsize',14);
% ylabel('y(t)','Fontsize',14);
% hold off

% Step Response
t1=linspace(0,10,100000);
y_fminunc= step(H_fminunc,t1);
y_ref= step(T,t1);
%y_reinforce = step(H{k_min},t1);

% % Step Response 
% plot(t1, y_fminunc,'LineWidth',1.8);
% hold on
% plot(t1, y_ref,'--','LineWidth',1.8);
% hold on
% plot(t1, y_reinforce,'--','LineWidth',1.8);
% title('Step Response','fontsize',14);
% legend('Fminunc System Step Response','Reference Step Response','Episodic REINFORCE System Step Response', 'fontsize',14);
% xlabel('t[s]','fontsize',14)
% hold off

% Step Response 
plot(t1, y_fminunc,'LineWidth',1.8);
hold on
plot(t1, y_ref,'--','LineWidth',1.8);
title('Step Response','fontsize',14);
legend('Fminunc System Step Response','Reference Step Response', 'fontsize',14);
xlabel('t[s]','fontsize',14)
hold off


%% Fminunc (reject disturbance)
tic
close all;

N=1;
idx=1;
J_vec=zeros(1,N);

for idx = 1:N
close all;

% Open-loop Unstable
[phi_final, J,K_controller, H,Q,output] = fminunc_disturb(u,t, phi_initial,alpha, T,P,s0_r0);

Sh= isstable(H);
if Sh == 0
    disp('Closed-loop unstable');
end

disp('Fminunc:');
disp(output);
disp(['Final vector of Phi: ', num2str(phi_final),]);
disp(['Final cost: ', num2str(J)]);

if N<2
    break
end

% Augment a dimension 
%close all;
phi_initial=[phi_final, 0];
disp(['Dimension:', num2str(length(phi_initial)),', Phi initial: ', num2str(phi_initial),]);

end
toc

% Metrics
T_sim=60;
out3=sim('tf_rocketmodel',T_sim);
theta_tf=out3.theta;

opt_controller=K_controller;
out4=sim('total_rocketmodel',T_sim);
theta_yp=out4.theta;

J_lqg_dist = sum((0-theta_tf).^2); % Sum of squared error
J_yp_dist =sum((0-theta_yp).^2);

ratio_opti_dist=(1-(J_yp_dist/J_lqg_dist))*100;

disp('Reject Disturbance Metrics: ')
disp(['cost LQG: ', num2str(J_lqg_dist),]);
disp(['cost YK Controller: ', num2str(J_yp_dist),]);
disp(['Optimization Ratio (%): ', num2str(ratio_opti_dist), '%']);

if J_yp_dist<J_lqg_dist
   disp('Controller was optimized to reject disturbance :)')
else 
   disp('Controller was NOT optimized to reject disturbance :(')
end

% System Response
figure;
plot(t, theta_tf);
hold on;
plot(t, theta_yp);
title('fminunc');
legend('Response of the LQG Controller','Response of the Optimized Controller');
xlabel('t(s)');
ylabel('theta (rad)');
hold off

%% State Space

[Ak,Bk,Ck,Dk]= tf2ss(K_controller.numerator{1},K_controller.denominator{1});
K_ss= ss(Ak,Bk,Ck,Dk);

%% Reduce the order of the controller K

K_red = reduce(K_controller_fminunc,'ErrroType','ncf','order',2);
K_red_reinforce = reduce(K_controller{k_min},'ErrroType','ncf','order',2);

% Poles and Zeros of reduced controller
figure;
pzmap(K_red);
title('Poles and Zeros of Reduced Controller Kred');

% frequency response 
figure;
bode(K_controller_fminunc, K_red,'r--', logspace(-10,10,100));
legend('K', 'Kred','fontsize',14);
set(findall(gcf, 'Type', 'Line'), 'LineWidth', 1.8);
set(findall(gcf, 'Type', 'Text'), 'FontSize', 14);

% frequency response 
figure;
bode(K_controller{k_min}, K_red_reinforce,'r--', logspace(-10,10,100));
legend('Episodic REINFORCE K', 'Kred','fontsize',14);
set(findall(gcf, 'Type', 'Line'), 'LineWidth', 1.8);
set(findall(gcf, 'Type', 'Text'), 'FontSize', 14);

% New cost 
%H_red= feedback(P,K_red);
H_red=minreal((K_red*P)/(1+K_red*P));
y_red= lsim(H_red,u,t);
ym= lsim(T,u,t);
J_red = sum((y_red - ym).^2);

% [NUM,DEN]= ss2tf(K_red.A,K_red.B,K_red.C,K_red.D);
% K_red=tf(NUM,DEN);

H_red_reinforce=minreal((K_red_reinforce*P)/(1+K_red_reinforce*P));
y_red_reinforce= lsim(H_red_reinforce,u,t);

figure;
plot(t, y_red,'LineWidth',1.8);
hold on
plot(t, ym,'LineWidth',1.8);
hold on
plot(t, y_red_reinforce,'--','LineWidth',1.8);
title('Response of Reduced Order Controllers','fontsize',14);
legend('Fminunc System Response','Reference Response','Episodic REINFORCE System Response', 'fontsize',14);
xlabel('t[s]','fontsize',14)
hold off

% Step Response
t1=linspace(0,5,100000);
y_redfminunc= step(H_red,t1);
y_redref= step(T,t1);
y_redreinforce = step(H_red_reinforce,t1);

% Step Response 
plot(t1, y_redfminunc,'LineWidth',1.8);
hold on
plot(t1, y_redref,'--','LineWidth',1.8);
hold on
plot(t1, y_redreinforce,'--','LineWidth',1.8);
title('Step Response of Reduced Order Systems','fontsize',14);
legend('Fminunc System Step Response','Reference Step Response','Episodic REINFORCE System Step Response', 'fontsize',14);
xlabel('t[s]','fontsize',14)
hold off

%% Remove Non minimum phase zeros
s= tf('s');
nmp_root= 66.0782;
gain= -195.6;
r_1=-24.9755;
r_2=-1.19;
r_3=-1;
r_4=-0.8232;
K_nmp_num= (gain*nmp_root)*(s-r_1)*(s-r_2)*(s-r_3)*(s-r_4);

K_nmp_den= tf(1,K_controller.denominator{1});

K_nmp= K_nmp_num*K_nmp_den;
H_nmp= minreal((K_nmp*P)/(1+K_nmp*P));

%% Mirror Non minimum phase zeros
s= tf('s');

K_nmp_num_2= (gain)*(s+24.9755)*(s+1.19)*(s+1)*(s+0.8232)*(s+nmp_root);

K_mirror_nmp= K_nmp_num_2*K_nmp_den;
H_mirror_nmp= minreal((K_mirror_nmp*P)/(1+K_mirror_nmp*P));

%% Controllers Performance Comparison (Simulink: Reject Disturbance)

%opt_controller=K_controller{k_min};
%opt_controller=SR;
opt_controller=K_controller;
T_sim=60;

% s0_r0
out3=sim('tf_rocketmodel',T_sim);
x_tf=out3.x;
z_tf=out3.z;
theta_tf=out3.theta;
t_tf=out3.ts;
gimbal_signal_tf=out3.gimbal;
%lintheta_tf=out3.theta_linear;

% YP
out4=sim('total_rocketmodel',T_sim);
x_yp=out4.x;
z_yp=out4.z;
theta_yp=out4.theta;
%lintheta_yp=out4.theta_linear;

distub_gimbal=out4.disturb;
gimbal_control=out4.Phi_control;
gimbal_signal_yp=out4.gimbal;
d_gimbal=out4.d_gimbal;

figure;
plot(t_tf,distub_gimbal,'linewidth',1.6);
xlabel('t [s]','FontSize', 14); 
ylabel('angle [rad]', 'FontSize', 14); 
title('Input Noise', 'FontSize', 14);

figure;
plot(x_tf,z_tf,'LineWidth',1.6);
hold on;
plot(x_yp,z_yp,'--','LineWidth',1.6);
xlim([-1,1]);
xlabel('x [m]','FontSize', 14); 
ylabel('z [m]', 'FontSize', 14); 
title('2D trajectory', 'FontSize', 14);
legend('LQG Controller','YK Controller','FontSize', 14);

traj_error= x_tf-x_yp;
figure;
plot(t_tf,traj_error,'LineWidth',1.6);
xlabel('t [s]','FontSize', 14); 
ylabel('x [m]', 'FontSize', 14); 
title('Difference between Horizontal trajectories', 'FontSize', 14);

figure;
plot(t_tf,x_tf,'LineWidth',1.6);
hold on;
plot(t_tf,x_yp,'--','LineWidth',1.6);
xlabel('t [s]','FontSize', 14); 
ylabel('x [m]', 'FontSize', 14); 
title('Horizontal Position', 'FontSize', 14);
legend('LQG Controller','YK Controller','Fontsize', 14);

figure;
plot(t_tf,z_tf,'LineWidth',1.6);
hold on;
plot(t_tf,z_yp,'--','LineWidth',1.6);
xlabel('t [s]','FontSize', 14); 
ylabel('z [m]', 'FontSize', 14); 
title('Vertical Position', 'FontSize', 14);
legend('LQG Controller','YK Controller','Fontsize', 14);

figure;
plot(t_tf,theta_tf,'LineWidth',1.6);
hold on;
plot(t_tf,theta_yp,'--','LineWidth',2);
title('Angle \theta', 'Interpreter', 'tex', 'Fontsize', 14);
xlabel('t[s]','FontSize', 14); 
ylabel('\theta [rad]','Interpreter', 'tex','Fontsize', 14); 
legend('LQG Controller','YK Controller','Fontsize',14);

% Control Gimbal Comparison
figure;
plot(t_tf,distub_gimbal,'LineWidth',1.6);
hold on;
plot(t_tf,gimbal_signal_tf,'LineWidth',1.6);
hold on;
plot(t_tf,gimbal_signal_yp,'--','LineWidth',1.8);
xlabel('t[s]','FontSize', 14); 
ylabel('Angle [rad]','FontSize', 14); 
legend('Gimbal Angle Disturbance', 'LQG Controlled Gimbal', 'YK Controlled Gimbal','fontsize',14);
title('Input Gimbal Angle','Fontsize', 14);
hold off

figure;
plot(t_tf,gimbal_signal_tf,'LineWidth',1.6);
hold on;
plot(t_tf,gimbal_signal_yp,'--','LineWidth',1.8);
xlabel('t[s]','FontSize', 14); 
ylabel('Angle [rad]','FontSize', 14); 
legend('LQG Controlled Gimbal', 'YK Controlled Gimbal','fontsize',14);
title('Input Gimbal Angle','Fontsize', 14);
hold off


% derivative of gimbal angle
figure;
plot(t_tf,rad2deg(d_gimbal),'LineWidth',1.6);
xlabel('t[s]','FontSize', 14); 
ylabel('Angle [deg]','FontSize', 14); 
title('Derivative of the Gimbal Angle','Fontsize', 14);

% Step Response 
t1=linspace(0,1,1000);
%y_k_ctr_initial= step(H{1},t1); % Step Response of CLS (K_controller)
%y_k_ctr= step(H{k_min},t1); % Step Response of CLS (K_controller)
y_k_ctr= step(H,t1);
%y_k_2=step(H_con,t1); % Step Response of CLS (K_controller obtained with penalization of inversion)
y_s0_r0= step(H_s0_r0,t1); % Step Response of CLS (LQG Controller) "Reference"

y_ctr=y_k_ctr;
figure;
plot(t1, y_ctr,'linewidth',1.6);
hold on
plot(t1, y_s0_r0,'LineWidth',1.6);
% hold on
% plot(t1, y_k_ctr_initial,'linewidth',1.6);
xlim([-0.01,1]);
title('Step Response','fontsize',14);
%legend('System Step Response with YK Controller','System Step Response with LQG Controller','Initial Controller','Fontsize', 14);
legend('System Step Response with YK Controller','System Step Response with LQG Controller','Fontsize', 14);
hold off

% Error 
theta_goal=0;
mse_lqg=sqrt(mean((theta_goal - theta_tf).^2)); % root mean squared error
disp(['RMSE (s0_r0): ', num2str(mse_lqg)]);

mse_yp=sqrt(mean((theta_goal - theta_yp).^2)); % root mean squared error
disp(['RMSE (Optimized): ', num2str(mse_yp)]);

cost_lqg= sum((theta_goal- theta_tf).^2);
disp(['cost (s0_r0): ', num2str(cost_lqg)]);
cost_yp= sum((theta_goal- theta_yp).^2);
disp(['cost (Optimized): ', num2str(cost_yp)]);

cost_x_lqg= sum((0- x_tf).^2);
disp(['cost X (s0_r0): ', num2str(cost_x_lqg)]);
cost_x_yp= sum((0- x_yp).^2);
disp(['cost X (YK): ', num2str(cost_x_yp)]);
ratio_x_cost=(1-(cost_x_yp/cost_x_lqg))*100;
disp(['Optimization X Ratio: ', num2str(ratio_x_cost)]);

ratio_cost=(1-(cost_yp/cost_lqg))*100;
disp(['Optimization Ratio: ', num2str(ratio_cost)]);

if mse_yp<mse_lqg
   disp('Controller was optimized to reject disturbance')
elseif mse_yp==mse_lqg
   disp('Controller has the same response as LQG to reject disturbance')
else 
   disp('Controller was NOT optimized to reject disturbance')
end

%% Controllers Performance Comparison (Simulink: Follow Reference)

close all;
t=t';
T_sim=60;

opt_controller= K_controller{k_min};
%opt_controller=K_controller;

% s0_r0
out3=sim('tf_rocketmodel_followref',T_sim);
x_tf=out3.x;
z_tf=out3.z;
theta_tf=out3.theta;
t_tf=out3.ts;
gimbal_control_tf=out3.Phi_control;
lintheta_tf=out3.theta_linear;
y_ref=out3.y_ref;

% YP
out4=sim('total_rocketmodel_followref',T_sim);
x_yp=out4.x;
z_yp=out4.z;
theta_yp=out4.theta;
gimbal_control_yp=out4.Phi_control;
lintheta_yp=out4.theta_linear;

% Step Response 
t1=linspace(0,3,1000);
y_T=step(T,t1); %  % Step Response of "Reference" Transfer function

y_k_ctr= step(H{k_min},t1); % Step Response of CLS (Optimized K_controller)
%y_k_con=step(H,t1); % Step Response of CLS (K_controller obtained with penalization of inversion)
% y_k_ctr_2= step(H_nmp,t1); %Step Response of CLS (K_controller without nonminimum phase zeros)
% y_k_ctr_3= step(H_mirror_nmp,t1); %Step Response of CLS (K_controller with mirrored nmp zeros)
% y_k_red= step(H_red,t1);

y_ctr=y_k_con;
y_s0_r0= step(H_s0_r0,t1); % Step Response of CLS (LQG Controller) "Baseline controller"

plot(t1, y_T);
hold on
plot(t1, y_ctr,'--', 'linewidth',1.9);
hold on
plot(t1, y_s0_r0);
title('Step Response');
legend('Reference Transfer Function Step Response','CL System Step Response with YK Controller','CL System Step Response with LQG Controller');
hold off

figure;
% plot(t,u);
% hold on;
plot(t_tf,theta_tf);
hold on;
plot(t_tf,theta_yp,'--', 'linewidth',2);
title('Angle \theta Comparison (Follow Reference)', 'Interpreter', 'tex', 'Fontsize', 14);
xlabel('t[s]','FontSize', 14); 
ylabel('\theta [rad]','Interpreter', 'tex','Fontsize', 14); 
legend('LQG Controller','YK Controller');

% Control Gimbal Sum
figure;
plot(t_tf,gimbal_control_tf,'linewidth',1.5);
hold on;
plot(t_tf,gimbal_control_yp,'--');
xlabel('t[s]','FontSize', 14); 
ylabel('Angle [rad]','FontSize', 14); 
legend('LQG Controller Output', 'Optimized Controller Output');
title('Gimbal Angle','Fontsize', 14);
hold off

ym=lsim(T,u,t);
yf=lsim(H{k_min},u,t);
%yf=lsim(H,u,t);

% nonlinear vs lsim
figure;
plot(t_tf,theta_yp,'o-');
hold on;
plot(t_tf,yf);
xlabel('t[s]','FontSize', 14); 
ylabel('Angle [rad]','FontSize', 14); 
legend('Nonlinear Theta (Optimized Controller)','lsim (Optimized Controller)');
title('Nonlinear System Response Vs Lsim','Fontsize', 14);
hold off

% linear vs lsim
figure;
plot(t_tf,lintheta_yp,'o-');
hold on;
plot(t_tf,yf);
xlabel('t[s]','FontSize', 14); 
ylabel('Angle [rad]','FontSize', 14); 
legend('Linearized Theta (Optimized Controller)','lsim (Optimized Controller)');
title('Linear System Response Vs Lsim','Fontsize', 14);
hold off

% linear T vs lsim
figure;
plot(t_tf,y_ref,'o-');
hold on;
plot(t_tf,ym);
xlabel('t[s]','FontSize', 14); 
ylabel('Angle [rad]','FontSize', 14); 
legend('2nd order system response','lsim (2nd order tf)');
title('Linear System Response Vs Lsim (T)','Fontsize', 14);
hold off

% Error 

% % root mean squared error
% mse_lqg=sqrt(mean((u-theta_tf).^2)); 
% disp(['RMSE (s0_r0): ', num2str(mse_lqg)]);
% mse_yp=sqrt(mean((u-theta_yp).^2)); 
% disp(['RMSE (Optimized): ', num2str(mse_yp)]);

% sum of squared error
cost_lqg=sum((theta_tf - y_ref).^2);
disp(['J(s0_r0): ', num2str(cost_lqg)]);
cost_yp=sum((theta_yp - y_ref).^2);
disp(['J(Optimized): ', num2str(cost_yp)]);
cost_norm=sum((y_ctr - y_T).^2);
disp(['J(Normalized): ', num2str(cost_norm)]);
cost_norm_baseline=sum((y_s0_r0 - y_T).^2);
disp(['J(Normalized baseline s0_ro): ', num2str(cost_norm_baseline)]);

if cost_yp<cost_lqg
   disp('Controller was optimized to follow ref :)')
else 
   disp('Controller was NOT optimized to follow ref')
end

% % linearized system cost
% cost_lqg_lin=sum((lintheta_tf - ym).^2);
% disp(['linear J(s0_r0): ', num2str(cost_lqg)]);
% cost_yp_lin=sum((lintheta_yp - ym).^2);
% disp(['linear J(Optimized): ', num2str(cost_yp)]);
% 
% if cost_yp_lin<cost_lqg_lin
%    disp('Controller was optimized to follow ref (LINEAR) :)')
% else 
%    disp('Controller was NOT optimized to follow ref (LINEAR)')
% end



%% Validate R

for R=[1e-6 1e-3 1 100]
    [num,den] = tfdata(P);
    [A,B,C,D] = tf2ss(num{1},den{1});
    B=B*C(2);
    C=[0,1];
    sys = ss(A,B,C,D);
    
    Q=C'*C;
    K = lqr(A,B,Q,R);
    
    [Lc,Cc]=size(C);
    G = eye(size(A)); %Gain of the process noise
    Qe = [0.1, 0;
            0,  1e-6]; %Variance of process errors
    Re = eye(Lc)*1e-6; %Variance of measurement errors
    
    L = lqe(A,G,C,Qe,Re);
    sys_lqg=-reg(sys,K,L); % negative feedback
    s0_r0=tf(sys_lqg);
    
    figure(1);
    rlocus(s0_r0*sys);
    hold on;
end

figure(1);
rlocus(s0_r0*sys);
R=[1e-6 1e-3 1 100];
legend(sprintf('R=%.6f',R(1)),sprintf('R=%.3f',R(2)),sprintf('R=%d',R(3)),sprintf('R=%d',R(4)));
