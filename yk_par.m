function [Q,K,H,cost_yp]= yk_par(phi,P,s0_r0,alpha)
% Youla Parameter 
s=tf('s');
Q=0;

for i = 1:length(phi)
Q = Q + phi(i)*((alpha/(s+alpha))^(i-1));
end
Q=minreal(Q);

[num_sys,den_sys] = tfdata(P);
[numK0,denK0] = tfdata(s0_r0);
[numQ,denQ] = tfdata(Q);

K= tf((conv(denQ{1},numK0{1})+conv(numQ{1},den_sys{1})),(conv(denQ{1},denK0{1})-conv(numQ{1},num_sys{1}))); % Controller 
SR=K;
assignin('base','SR',SR);

H=minreal((K*P)/(1+K*P));
%H=minreal(P/(1+K*P));

out=sim('fminunc_rocketmodel',60);

x_f=out.x;
z_f=out.z;
theta_f=out.theta;
t_f=out.ts;
theta_reference=0;

% % 2D controller trajectory
% figure;
% plot(x_f,z_f)
% xlim([-1,1]);
% xlabel('x [m]','FontSize', 14); 
% ylabel('z [m]', 'FontSize', 14); 
% title('Controlled 2D trajectory', 'FontSize', 14);
% 
% % 3D
% figure;
% plot3(x_f,zeros(size(x_f)),z_f)
% xlim([-1,1]);
% xlabel('x [m]','FontSize', 14); 
% zlabel('z [m]', 'FontSize', 14); 
% title('3D trajectory','FontSize', 14);
% 
% % Controlled Theta
% figure;
% plot(t_f,theta_f);
% title('Angle \theta', 'Interpreter', 'tex', 'Fontsize', 14);
% xlabel('t[s]','FontSize', 14); 
% ylabel('\theta [rad]','Interpreter', 'tex','Fontsize', 14); 


cost_yp=sum((theta_reference - theta_f).^2) 
disp(['cost: ', num2str(cost_yp)]);

end