
%% f(phi_0)

% Initial Parameters
alpha = 1;

%phi_0= linspace(-1,1,21); % Parameter phi
phi_0= linspace(-1020,-1000, 100); % Parameter phi

% Preallocation 
J= zeros(1,length(phi_0));

Q= cell(1,length(phi_0));
H= cell(1,length(phi_0));
K= cell(1,length(phi_0));

% Ideal Transfer Function T(s) and Plant P(s)
flag_rocket=0;
count_inv=0;

[T,P,flag_rocket]=rocket_system;

[num_sys,den_sys] = tfdata(P);

% Input Signal 

[u,t] = sum_squares_sig();
Ts_sig=0.001;

% s0/r0
[s0_r0,Nbar,sys] = controller_design(P,flag_rocket);
[num,den] = tfdata(s0_r0);

% Square wave response
ym= lsim(T,u,t);

% Matrix Q: Finite Dimension Approximation 
s=tf('s');

for i = 1:length(phi_0)
Q{i} = phi_0(i)*((alpha/(s+alpha))^(0));

Q{i}=minreal(Q{i});
[numQ,denQ] = tfdata(Q{i});

K{i}= tf((conv(denQ{1},num{1})+conv(numQ{1},den_sys{1})),(conv(denQ{1},den{1})-conv(numQ{1},num_sys{1})));
SR=K{i};
out=sim('fminunc_rocketmodel',60);
theta_f{i}=out.theta;
t_f=out.ts;
theta_reference=0;

H{i}=minreal((K{i}*P)/(1+K{i}*P));

% Transfer Function (square wave response)
y{i}= lsim(H{i},u,t);

% t2=linspace(0,1,1000);
% y_2= step(H{i},t2); % Step Response of CLS (K_controller)
% y_ref= step(T,t2);
% J_inv= sign(y_2.*y_ref); % if sign is negative, penalize
% 
% count_inv = sum(J_inv == -1);

% Cost J as a function of parameter phi

%J(i) = sum((theta_reference-theta_f{i}).^2)+(0)*count_inv+ (0)*sum((y{i} - ym).^2); % Sum of squared error
J(i) = sum((theta_reference-theta_f{i}).^2);
disp(['phi= ', num2str(phi_0(i)), ',','cost= ', num2str(J(i))]); 

end

[Max_J,I_M] = max(J);
[min_J,I_m] = min(J);
disp(['Max cost is: ', num2str(Max_J), ' , with index: ', num2str(I_M)]);
disp(['Min cost is: ', num2str(min_J), ' , with index: ', num2str(I_m)]);

figure;
plot(phi_0,J,'LineWidth',1.6);
hold on;
plot(phi_0(I_m), min_J, 'rx', 'MarkerSize', 10, 'LineWidth', 1.5);
title('Cost as a function of parameter phi','fontsize', 14);
xlabel('phi','fontsize', 14);
ylabel('Cost','fontsize', 14);
legend('Cost', 'Minimum Cost','fontsize', 14);

%% Verif

phi_f=phi_0(I_m);
J_f= cost_disturb(phi_f);
disp(['Min cost with Cost_disturb is: ', num2str(J_f)]);

%% Yp
for phi=[-1050,-1012.5,-800,100]

[Q,SR,H,cost_yp]= yk_par(phi,P,s0_r0,alpha);

figure(1);
pzmap(SR);
% Get the handle of the current axes
ax = gca;
% Set the linewidth of the lines in the plot
lineWidth = 2; % Desired linewidth
set(findobj(ax, 'Type', 'line'), 'LineWidth', lineWidth);
hold on;
end

figure(1);
title('Pole-Zero Plot of the Controller','fontsize', 14);
xlabel('Real','fontsize', 14);
ylabel('Imaginary','fontsize', 14);
phi=[-1050,-1012.5,-800,100];
%phi=[-1020,-1012.5,-800,800];o
%phi=[-1013,-1010,-1012.5,800];
legend(sprintf('phi= %0.1f',phi(1)),sprintf('phi= %0.1f',phi(2)), sprintf('phi= %0.1f',phi(3)), sprintf('phi= %0.1f',phi(4)),'location', 'Northwest', 'FontSize', 14);
%,sprintf('phi= %0.1f',phi(5)),sprintf('phi= %0.1f',phi(6))
hold off
