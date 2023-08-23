function J = cost(phi)

[T,P]=second_order();
%[T,P]=first_order();

% Input Signal
[u,t] = sum_squares_sig();

ym= lsim(T,u,t); % square wave response

alpha=15;
% alpha= -10*ones(1,5);

s=tf('s');

% Matrix Q: Finite Dimension Approximation 
Q = 0;
% Q=1;
% A=1;
% B=1;

for i = 1:length(phi)
Q = Q + phi(i)*((alpha/(s+alpha))^(i-1));
% A = A*(phi(i)*s^(i-1));
% B = B*(s-alpha(i));
end
Q=minreal(Q);

% Q = A/B;

% Closed-loop Transfer Function  H = P(1 + Q*P);
H= minreal(P*(1 + Q*P)); 
% sysr = minreal(H);

% Set of stabilizing controllers K= Q / (1+Q*P); 
K= minreal(Q/(1+Q*P));

% Transfer Function (square wave response)
y= lsim(H,u,t);

% Cost Function J(T, Q(Phi))

J = sum((y - ym).^2); % Sum of squared error
   
end