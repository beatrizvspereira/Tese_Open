function J = cost_disturb(phi)

[T,P, flag_rocket]=rocket_system();
%[T,P]=second_order();

[s0_r0,Nbar,K,E] = controller_design(P,flag_rocket);

% Input Signal
[u,t] = sum_squares_sig();

theta_reference=0;

alpha=1;

s=tf('s');

% Matrix Q: Finite Dimension Approximation 
Q = 0;

for i = 1:length(phi)
Q = Q + phi(i)*((alpha/(s+alpha))^(i-1));
end
disp(['Phi: ', num2str(phi),]);
Q=minreal(Q);

[num_sys,den_sys] = tfdata(P);
[numK0,denK0] = tfdata(s0_r0);
[numQ,denQ] = tfdata(Q);

% Set of stabilizing controllers
K= tf((conv(denQ{1},numK0{1})+conv(numQ{1},den_sys{1})),(conv(denQ{1},denK0{1})-conv(numQ{1},num_sys{1}))); % Controller 
SR=K;
assignin('base','SR',SR);

out=sim('fminunc_rocketmodel',60);
theta_f=out.theta;

% Cost Function J(T, Q(Phi))

J = sum((theta_reference - theta_f).^2); % Sum of squared error
disp(['cost: ', num2str(J),]);
   
end