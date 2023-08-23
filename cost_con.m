function J = cost_con(phi)

% weight4
c1=1; %weight that penalizes deviations from reference
c2=0; %weight that penalizes deviations from goal angle
c_inv= 1e8; %weight that penalizes invertion in step response (non minimum phase zeros)

count_inv=0;
theta_reference=0;
flag_rocket=0;

%system
%[T,P,flag_rocket]=rocket_system();
[T,P]=second_order();

%baseline controller design
[s0_r0,~] = controller_design(P,flag_rocket);

% Input Signal
[u,t] = sum_squares_sig();

ym= lsim(T,u,t); % square wave response

alpha=10;

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

% theta angle
% SR=K;
% assignin('base','SR',SR);
% out=sim('fminunc_rocketmodel',60);
% theta_f=out.theta;

% Closed-loop Transfer Function (Negative Feedback)
H=minreal((K*P)/(1+K*P)); %Feedforward

% Transfer Function (square wave response)
y= lsim(H,u,t);

% Step Response
t2=linspace(0,1,1000);
y_2= step(H,t2); % Step Response of CLS (K_controller)
y_ref= step(T,t2);

% Cost Function J(T, Q(Phi))
%J_dist = sum((theta_reference - theta_f).^2); % Sum of squared error
J_fref = sum((y - ym).^2); % Sum of squared error
J_inv= sign(y_2.*y_ref); % if sign is negative, penalize

count_inv = sum(J_inv == -1);

% if isempty(find(J_inv <0)) == 1
%     c_inv=0.1;
% else 
%     c_inv=100;
% end    
    
%J= c1*J_fref + c2*J_dist + c_inv*count_inv;
J= c1*J_fref + c_inv*count_inv;
%J= c1*J_fref + c2*J_dist;

%disp(['cost: ', num2str(J),]);
   
end