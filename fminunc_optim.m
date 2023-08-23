function [phi_final, J, J_norm, output,K,H] = fminunc_optim(u,t, phi_initial,alpha,T,P)

% Optimization
fun= @cost;

[phi_final,J,~,output] = fminunc(fun,phi_initial);

% Youla Parameter 
s=tf('s');
Q=0;
% Q=1;
% A=1;
% B=1;

for i = 1:length(phi_final)
Q = Q + phi_final(i)*((alpha/(s+alpha))^(i-1));
% A = A*(phi_final(i)*s^(i-1));
% B = B*(s-alpha(i));
end
Q=minreal(Q);

% Q= A/B;

H= minreal(P*(1 + Q*P)); % Closed-loop Transfer Function
K= minreal(Q/(1+Q*P)); % Controller 

% Step Response
t1=linspace(0,5,100);
y_sis= step(H,t1);
y_ref= step(T,t1);

% System Response 
ym= lsim(T,u,t);
yf= lsim(H,u,t);

% Normalized cost function 
J_norm = sum((y_sis - y_ref).^2);

% Plots

% Step Response 
plot(t1, y_sis);
hold on
plot(t1, y_ref);
title('Step Response');
legend('System Step Response','Ideal Step Response');
hold off

% System Response
figure;
plot(t, ym);
hold on;
plot(t, yf);
title('fminunc');
legend('Reference','Response of the Controller');
xlabel('t(ms)');
ylabel('y(t)');
hold off

% plot of poles and zeros 
figure;
pzmap(K);
title('Poles and Zeros of Controller K');

% frequency response 
figure;
bode(K, logspace(-10,10,100));
title('Bode Diagram of Controller K');

end