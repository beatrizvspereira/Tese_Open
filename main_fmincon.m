%%  Main: Automatic design of Controller Parameters based on YP and fmincon
% Beatriz Ventura dos Santos Pereira
% Nº90029

%% Input

% Signal
[u,t] = sum_squares_sig();
Ts_sig=0.001;

% System
flag_rocket=0;
%[T,P,flag_rocket]=rocket_system();
[T,P]=second_order();

% Initial Vector Phi
phi_initial=-80;

% Alpha
alpha= 10;

% Noise seed
rng(100);

%% Initial LQG controller S0/R0

[s0_r0,Nbar,sys] = controller_design(P,flag_rocket);
figure;
step(s0_r0)
H_s0_r0=minreal((s0_r0*P)/(1+s0_r0*P));
CL_stability=isstable(H_s0_r0) %Negative Feedback

%% Fmincon ("follow ref" control structure)

tic

close all;

A = [];
b = [];
Aeq = [];
beq = [];
lb = [];
ub = [];
nonlcon = [];

fun= @cost_LQ;

%[phi_final,J] = fmincon(fun,phi_initial,A,b,Aeq,beq,lb,ub,nonlcon);

options.Algorithm = 'sqp';
[phi_final,J] = fmincon(fun,phi_initial,[],[],[],[],0,[],[],options);

Sh= isstable(H);
if Sh == 0
    disp('Closed-loop unstable');
end

disp('Fmincon:');
disp(output);
disp(['Final vector of Phi: ', num2str(phi_final),]);
disp(['Final cost: ', num2str(J)]);

toc

%% fminunc with conditions

% Optimization
fun= @cost_con;

%phi_initial= [195.554431      17279.7621      10572.0886];
phi_initial=[-900,1.4e-6,1.6e-6];
%phi_initial=[phi_final,0];

options = optimoptions('fminunc');
options.StepTolerance=1e-12;
options.Display='iter-detailed';
options.FiniteDifferenceStepSize=1e-2;
options.OptimalityTolerance=1e-20;

[phi_final,J,~,output] = fminunc(fun,phi_initial,options);

s=tf('s');
Q=0;
count_inv=0;

% Youla Parameter
for i = 1:length(phi_final)
Q = Q + phi_final(i)*((alpha/(s+alpha))^(i-1));
end
Q=minreal(Q);

[num_sys,den_sys] = tfdata(P);
[numK0,denK0] = tfdata(s0_r0);
[numQ,denQ] = tfdata(Q);

% Controller 
K_controller_con= tf((conv(denQ{1},numK0{1})+conv(numQ{1},den_sys{1})),(conv(denQ{1},denK0{1})-conv(numQ{1},num_sys{1}))); % Controller 

% Closed-loop System
H_con=minreal((K_controller_con*P)/(1+K_controller_con*P));

% Step Response
t2=linspace(0,1,1000);
y_2= step(H_con,t2); % Step Response of CLS (K_controller)
y_ref= step(T,t2);

J_inv= sign(y_2.*y_ref); % if sign is negative, penalize
count_inv = sum(J_inv == -1)

figure;
step(H_con,t2);
hold on
step(T,t2);
hold off
legend('CL system Step Response', 'Reference Step Response');

disp('roots of controller: ');
roots(K_controller_con.numerator{1})
disp('poles of controller: ');
pole(K_controller_con)

t3=linspace(0,800,1000);
step(H_con,t3); % Step Response of CLS (K_controller_con)
