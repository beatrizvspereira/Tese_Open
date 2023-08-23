%%  Main: Automatic design of Controller Parameters based on YP and REINFORCE
% Beatriz Ventura dos Santos Pereira
% Nº90029

%% Input

% Input Signal
[u,t] = sum_squares_sig();
Ts_sig=0.001;

% System (Plant and Reference System)
flag_rocket=0;

[T,P,flag_rocket]=rocket_system();
%[T,P]=second_order();
%[T,P]=higher_order();

% Initial Vector Phi Phi_0
%phi_initial=[-0.1, 0.3,0.04,-0.07];
phi_initial=-850;
%phi_initial=[phi_final, 0];

% Alpha
alpha= 1;

% Noise seed 
rng(100);

% Learning parameter
eta= 10;

% Max. number of iterations
N =1500;

% Max. cost error allowed (stop condition)
max_e = 1e-11;


%% Initial LQG controller S0/R0

[s0_r0,Nbar,sys] = controller_design(P,flag_rocket);
figure;
step(s0_r0)
H_s0_r0=minreal((s0_r0*P)/(1+s0_r0*P));
CL_stability=isstable(H_s0_r0) %Negative Feedback

%% REINFORCE Algorithm
tic
close all;

theta_reference=0;
% cost function weights
c1=0; %weight that penalizes deviations from reference
c2=1; %weight that penalizes deviations from goal angle
c_inv=0;

[phi_final, J, J_norm, k, K_controller,H,Q,J_min,k_min,phi_min,v_eta,step_size] = reinforce_optim(N,eta, phi_initial, max_e, u,t, alpha,T,P,s0_r0,theta_reference,c1,c2,c_inv);

S= isstable(K);
disp(' REINFORCE:');
disp(['Final vector of Phi: ', num2str(phi_final),]);
disp(['Final cost: ', num2str(J), ' , Normalized cost: ', num2str(J_norm)]);
disp('Min value of J: ');
disp(J_min);

toc

% Save Control Data
data_file = matfile('47_data.mat','Writable',true);
save('47_data.mat','K_controller','H','Q');


%% Compare learning curves

load('rlcost_47.mat')
J_47= J;
J_47_min= J_min;

load('rlcost_31.mat')
J_31= J;
J_31_min= J_min;

figure;
plot(J_47,'linewidth',1.8);
hold on;
plot(J_31,'--','linewidth',1.8);
set(gca,'yscale','log');
title('Learning Curve','Fontsize', 18);
ax = gca;
ax.XAxis.FontSize = 18;
ax.YAxis.FontSize = 18;
ylim([1e-11,1e-5]);
legend('Vanilla Approach','New Convergence Approach', 'Fontsize', 18);
xlabel('Iterations','Fontsize', 18);
ylabel('Cost','Fontsize', 18);