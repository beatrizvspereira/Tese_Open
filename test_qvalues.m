[T,P, flag_rocket]=rocket_system();

phi=linspace(0.1,1,4);
alpha=1;

% Plant
s=tf('s');

Q=tf([0,-1000],[0,1]);
[numQ,denQ] = tfdata(Q);

[s0_r0,Nbar,sys] = controller_design(P,flag_rocket);

[num_sys,den_sys] = tfdata(P);
[num,den] = tfdata(s0_r0);

K= tf((conv(denQ{1},num{1})+conv(numQ{1},den_sys{1})),(conv(denQ{1},den{1})-conv(numQ{1},num_sys{1})));

H=minreal((K*P)/(1+K*P));
isstable(H)

%% Roberts
%[T,P, flag_rocket]=rocket_system();
[T,P]=second_order();

phi=linspace(0.1,1,4);
alpha=1;

% Plant
s=tf('s');

% Q= s/(s+1); % Stable, 1 pole
% Q_1= (s+0.01)/(s+0.02); % Stable (dominant poles)
% Q_2= (s+1)/(s+2); % Stable (faster dynamics)
% 
omega =20;
ksi = 0.1; % oscillatory
Q_2 = tf(omega^2, [1, 2*ksi*omega, omega^2]); 

%Q_1=tf([0,-1000],[0,1]); % static gain
Q_1= phi(1)*(s/(s-1));
%Q_2= phi(2)*(((10^-0.5)/(s+(10^-0.5))));
%Q_3= phi(3)*(s/((s+(0.5+0.1i))*(s+(0.5-0.1i))));
Q_3= phi(3)*(s/((s+0.5)*(s+0.2)));
Q_4= phi(4)*((0.001*s/(s+0.001)));

% % Closed-loop Transfer Function  H = P(1 + Q*P);

H_1= P*(1 + Q_1*P); 
H_2=P*(1 + Q_2*P);
H_3=P*(1 + Q_3*P);
H_4= P*(1 + Q_4*P); 

isstable(H_1)
isstable(H_2)
isstable(H_3)

% Set of stabilizing controllers K= Q / (1+Q*P); 
K_1= Q_1/(1+Q_1*P);
K_2= Q_2/(1+Q_2*P);
K_3= Q_3/(1+Q_3*P);
K_4= Q_4/(1+Q_4*P);

% subplot(2,2,1)
% set(gca,'FontSize',14)
% step(H_1)
% title('Youla Parameter Q_1(s)')

subplot(3,1,1)
set(gca,'FontSize',14)
step(H_2)
set(findall(gcf,'Type','line'),'LineWidth',1.5);
title('Youla Parameter Q_1(s)')

subplot(3,1,2)
set(gca,'FontSize',14)
step(H_3)
set(findall(gcf,'Type','line'),'LineWidth',1.5);
title('Youla Parameter Q_2(s)')

subplot(3,1,3)
set(gca,'FontSize',14)
step(H_4)
set(findall(gcf,'Type','line'),'LineWidth',1.5);
title('Youla Parameter Q_3(s)')
