[T,P]=second_order();
%[T,P]=first_order();
[u,t] = sum_squares_sig();

[s0_r0,Nbar] = controller_design(P);
%s0_r0=sim_controller_design(P);
%s0_r0= tf(0,[1,1]);

%Q= tf(1,[1,1]);
%Q=tf(-99.96,1);

%%

% Set of stabilizing conffltrollers K= Q / (1+Q*P); 
K= minreal(Q/(1+Q*P));

% Closed-loop Transfer Function  H = P(1 + Q*P);
H= minreal(P*(1 + Q*P));
%H_1=feedback(P, K,+1);
%H= minreal(P/(1 - P*K)); 

%%
[num_sys,den_sys] = tfdata(P);
[num,den] = tfdata(s0_r0);
[numQ,denQ] = tfdata(Q);

% Set of stabilizing controllers K= Q / (1+Q*P) or  K= (Sº+Q*A) / (Rº + Q*B); 
K_2= (tf((conv(denQ{1},num{1})+conv(numQ{1},den_sys{1})),(conv(denQ{1},den{1})-conv(numQ{1},num_sys{1}))));

figure;
step(s0_r0);
title('LQR+Observer controller Step Response');

figure;
step(K_2);
title('Stabilizing Controller Step Response');

% Closed-loop Transfer Function  H = P(1 + Q*P);

% h_d= 1+P*K_2;
% H_man= P/h_d; 
% H_21=minreal(H_man);

%H_2=feedback(P,K_2);
%C= tf(1,[1,0]);
%H_3=feedback(C*P,K_2);
H_2=minreal((K_2*P)/(1+K_2*P));
damp(H_2);

figure;
step(T);
hold on;
step(H_2);
legend('Goal TF','Closed loop system');
title('Closed-loop Transfer function (with LQR+0bserver) Step Response');

ym= lsim(T,u,t);
yf= lsim(H_2,u,t);
figure;
plot(t, ym);
hold on;
plot(t, yf);
title('fminunc');
legend('Reference','Response of the Controller');
xlabel('t(ms)');
ylabel('y(t)');
hold off

