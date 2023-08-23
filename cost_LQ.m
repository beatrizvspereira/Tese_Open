function J = cost_LQ(phi)

flag_rocket=0;

%[T,P, flag_rocket]=rocket_system();
%[T,P]=second_order();
[T,P]=higher_order();

[s0_r0,Nbar,K,E] = controller_design(P,flag_rocket);

% Input Signal
[u,t] = sum_squares_sig();
%[u,t] = square_sig();
%[u,t]=prbs_sig();

ym= lsim(T,u,t); % square wave response

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

% Closed-loop Transfer Function (Negative Feedback)
%H= minreal(P/(1 + P*K)); %Feedback
%H= feedback(P,K);
H=minreal((K*P)/(1+K*P)); %Feedforward

%C= tf(1,[1,0]);
%H= feedback(P*C,K);

% Transfer Function (square wave response)
y= lsim(H,u,t);

% Cost Function J(T, Q(Phi))

J = sum((y - ym).^2); % Sum of squared error
disp(['cost: ', num2str(J),]);
   
end