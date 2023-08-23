function [phi_final, J, J_norm, output,K, H,Q] = fminunc_LQ(u,t, phi_initial,alpha, T,P,s0_r0)

% Optimization
fun= @cost_LQ;

options = optimoptions('fminunc');
options.StepTolerance=1e-12;
options.Display='iter-detailed';
options.FiniteDifferenceStepSize=1e-3;
options.OptimalityTolerance=1e-12;

[phi_final,J,~,output] = fminunc(fun,phi_initial,options);

% Youla Parameter 
s=tf('s');
Q=0;

for i = 1:length(phi_final)
Q = Q + phi_final(i)*((alpha/(s+alpha))^(i-1));
end
Q=minreal(Q);

[num_sys,den_sys] = tfdata(P);
[numK0,denK0] = tfdata(s0_r0);
[numQ,denQ] = tfdata(Q);

K= tf((conv(denQ{1},numK0{1})+conv(numQ{1},den_sys{1})),(conv(denQ{1},denK0{1})-conv(numQ{1},num_sys{1}))); % Controller 
%H= minreal(P/(1 + P*K)); % Closed-loop Transfer Function
H=minreal((K*P)/(1+K*P)); % Feedforward
%H= feedback(P,K); % Closed-loop Transfer Function

% Step Response
t1=linspace(0,100,100000);
y_sis= step(H,t1);
y_ref= step(T,t1);

% System Response 
ym= lsim(T,u,t);
yf= lsim(H,u,t);

% Normalized cost function 
J_norm = sum((y_sis - y_ref).^2);

% Plots

% Step Response 
plot(t1, y_sis,'LineWidth',1.8);
hold on
plot(t1, y_ref,'--','LineWidth',1.8);
%xlim([-0.1,1]);
title('Step Response','fontsize',14);
legend('Fminunc System Step Response','Reference Step Response','fontsize',14);
hold off

% System Response
figure;
plot(t, ym,'LineWidth',1.8);
hold on;
plot(t, yf,'--', 'LineWidth',1.8);
title('Fminunc Results','fontsize',14);
legend('Reference','Response of the Fminunc Controller','fontsize',14);
xlabel('t(ms)','fontsize',14);
ylabel('y(t)','fontsize',14);
hold off

% plot of poles and zeros 
figure;
pzmap(K);
title('Poles and Zeros of Controller K');
figure;
pzmap(H);
title('Poles and Zeros of Closed loop H');

% frequency response 
figure;
bode(K, logspace(-10,10,100));
title('Bode Diagram of Controller K');

end