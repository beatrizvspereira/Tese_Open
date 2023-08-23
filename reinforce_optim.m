function [phi_final, J, J_norm, k, K,H,Q_f,J_min,k_min,phi_min,v_eta,step_size] = reinforce_optim(N,eta, phi_initial, max_e, u,t, alpha,T,P,s0_r0,theta_reference,c1,c2,c_inv)

% System Response 
ym= lsim(T,u,t); 
y_ref= step(T,t);

% Q-Design and REINFORCE
[v_phi,y,J,k, K,yf,H,Q_f,J_min,k_min,phi_min,v_eta,step_size] = Q_design(P,N,phi_initial,ym, y_ref, max_e,eta,u,t,alpha,s0_r0, theta_reference,c1,c2,c_inv);

% Final vector of phi
phi_final= v_phi(k,:);

% Step Response
y_sis= step(H{k},t);

% Normalized cost function 
J_norm = sum((y_sis - y_ref).^2);

% Plots

% plot of the last iteration 
figure;
plot(t, ym,'LineWidth',1.8);
hold on;
plot(t, yf,'LineWidth',1.8);
hold on
plot (t,u,'LineWidth',1.8);
hold off
title('Last Iteration','FontSize',14);
legend('Response of the Goal Transfer Function (Reference)','Response of the Controller', 'Input Signal','FontSize',14);
xlabel('t(ms)','FontSize',14);
ylabel('y(t)','FontSize',14);


% plot of the last iteration 
figure;
plot(t, ym,'LineWidth',1.8);
hold on;
plot(t, yf,'LineWidth',1.8);
hold off
title('Last Iteration','FontSize',14);
legend('Response of the Goal Transfer Function (Reference)','Response of the Controller','FontSize',14);
xlabel('t(ms)','FontSize',14);
ylabel('y(t)','FontSize',14);

% plot of the step response
figure;
plot(t, y_sis);
hold on
plot(t, y_ref);
title('Step Response','FontSize',14);
legend('System Step Response','Ideal Step Response','FontSize',14);
hold off

opt_controller=s0_r0;
out_tf=sim('tf_rocketmodel',60);
theta_tf=out_tf.theta;
cost_lqg= sum((0- theta_tf).^2);

figure;
plot(J,'linewidth',1.8);
%set(gca,'xscale','log');
set(gca,'yscale','log');
title('Learning Curve','Fontsize', 14);
legend('Cost Function','Fontsize', 14);
xlabel('Iterations','Fontsize', 14);
ylabel('Cost','Fontsize', 14);

if c2>0
figure;
plot(J,'linewidth',1.8);
hold on
yline(cost_lqg,'r--','LineWidth',1.6);
%set(gca,'xscale','log');
set(gca,'yscale','log');
%ax = gca;
%ax.XAxis.FontSize = 18;
%ax.YAxis.FontSize = 18;
title('Learning Curve','Fontsize', 14);
legend('YK Controller', 'LQG Controller', 'Fontsize', 14,'Location', 'south west');
xlabel('Iterations','Fontsize', 14);
ylabel('Cost','Fontsize', 14);
end 

% plot of the dynamic step 
figure;
plot(v_eta,'linewidth',1.8);
hold on
plot(J,'linewidth',1.8);
set(gca,'yscale','log');
title('learning rate vs cost','Fontsize', 14);
legend('Learning rate','Cost','Fontsize', 14);
xlabel('Iterations','Fontsize', 14);
hold off

dyn_step= zeros(1,N);

for it= 1:N
dyn_step(it)= v_eta(it)/J(it);
end

figure;
plot(dyn_step,'linewidth',1.8);
set(gca,'yscale','log');
title('Episodic REINFORCE Step Gain \eta/J(\phi_i)', 'Interpreter', 'tex', 'Fontsize', 14);

figure;
plot(step_size,'linewidth',1.8);
ylabel('Step size','FontSize',14);
xlabel('Iterations','FontSize',14);
title('Episodic REINFORCE Step', 'Fontsize', 14);

% % plot of the root locus
% figure;
% rlocus(H{k},'b',T,'k')
% hold on
% legend('H','T')
% hold off
% 
% % plot of poles and zeros of controller 
% figure;
% pzmap(K{k});
% title('Poles and Zeros of Reduced Controller K');
% 
% % frequency response 
% figure;
% bode(K{k}, logspace(-10,10,100));
% title('Bode Diagram of Controller K');

end