%%  Q-Design
% Beatriz Ventura dos Santos Pereira
% Nº90029

%% Input Signal

%  [u,t] = square_sig();
[u,t] = sum_squares_sig();
%[u,t] = prbs_sig();

%% Parameters 

% System Transfer Functions

%[T,P]=first_order();
%[T,P]=second_order();
[T,P]=rocket_system();

% Initial Controller LQR+observer S0/R0
[s0_r0,Nbar] = controller_design(P);

figure;
step(s0_r0);
CL_stability=isstable(minreal((s0_r0*P)/(1+s0_r0*P)));

% Initial Vector Phi
phi_initial=-0.5;
% rng(100);
% phi_initial=randn(1,2);

% Learning parameter
eta= 10;

% Alpha
alpha= 15;

% Max. number of iterations
N = 80000;

% Noise 
rng(1000);

% Max. cost error allowed (stop condition)
max_e = 0.01;

%% Fminunc
tic
close all;
%%Open-loop Stable
% [phi_final, J, J_norm,output, K,H] = fminunc_optim(u,t, phi_initial,alpha,T,P);

%%Open-loop Unstable
[phi_final, J, J_norm,output, K_controller,H,Q] = fminunc_LQ(u,t, phi_initial,alpha,T,P,s0_r0);

%Sk= isstable(K_controller);
Sh= isstable(H);
if Sh == 0
    disp('Closed loop unstable');
end

% [Gm,Pm,Wcg,Wcp] = margin(K);

disp('Fminunc:');
disp(output);
disp(['Final vector of Phi: ', num2str(phi_final),]);
disp(['Final cost: ', num2str(J), ' , Normalized cost: ', num2str(J_norm)]);
toc
%% Augment a dimension 

phi_initial=[phi_final, 0];
disp(['Dimension:', num2str(length(phi_initial)),', Phi initial: ', num2str(phi_initial),]);

%% REINFORCE Algorithm

[phi_final, J, J_norm, k, K,H,Q] = reinforce_optim(N,eta, phi_initial, max_e, u,t, alpha,T,P,s0_r0);

S= isstable(K);
disp('REINFORCE:');
disp(['Final vector of Phi: ', num2str(phi_final),]);
disp(['Final cost: ', num2str(J), ' , Normalized cost: ', num2str(J_norm)]);

%% Reduce the order of the controller K

K_red = reduce(K,'ErrroType','ncf','order',10);
% K_red = reduce(K{k},'ErrroType','ncf','order',5);

% Poles and Zeros of reduced controller
figure;
pzmap(K_red);
title('Poles and Zeros of Reduced Controller Kred');

% frequency response 
figure;
bode(K, K_red, logspace(-10,10,100));
%bode(K{k}, K_red, logspace(-10,10,100));
legend('K', 'Kred');

% New cost 
H_red= feedback(P,K_red);
y_red= lsim(H_red,u,t);
ym= lsim(T,u,t);
J_red = sum((y_red - ym).^2);

%% Step Responses

% Q_2= tf(0,1);
% 
% [num_sys,den_sys] = tfdata(P);
% [num,den] = tfdata(s0_r0);
% [numQ,denQ] = tfdata(Q_2);

% K_lq= (tf((conv(denQ{1},num{1})+conv(numQ{1},den_sys{1})),(conv(denQ{1},den{1})-conv(numQ{1},num_sys{1}))));
%H_lq=feedback(P,s0_r0);

step(T);
hold on;
step(H);
hold on;
step(s0_r0);
% % hold on;
% % step(H_red,'y--');
legend('Ideal TF Step Response','Closed-loop TF Step Response','Step Response of LQR+Observer controller');
hold off;


%% Learning progress

idx=1;
%idx=idx+1;
J_vec(idx)=J;

disp(['Learning: ', num2str(J_vec),]);

% plot(J_vec);
% set(gca,'yscale','log');
% title('Learning in function of dimension');
% legend('learning curve');
% xlabel('N');
% ylabel('J');
