function [v_phi,y,J,k, K,yf,H,Q_f]= Roberts_Q_design(P,N,phi_initial,ym, max_e,eta,u,t,alpha)

v_phi= zeros(N, length(phi_initial));
v_phi(1,:)= phi_initial;

% Plant
s=tf('s');

J= zeros(1,N);
count=0;

f = figure;
filename = 'follow_ref.gif';

% Adjustable parameters

tolerance = 0.001;

min_eta = 0.1; % min. value accepted for eta

J_rel = 0.1; % learning cost should improve over 10%

J_th = 0.02; % arbitrary small value of cost

J_rel_min = 0.01; % learning cost should improve a min. of 1% 
% when cost is arbitrarily small

var= 0.001; % factor multiplied but the noise 

for k = 1:N

% Matrix Q: Finite Dimension Approximation 
Q = 1;
A= 1;
B=1;

for i = 1:length(v_phi(k,:))
A = A*(v_phi(k,i)*s^i);
B = B*(s-alpha(i));
end

Q= A/B;

Q_f{k}=Q;

% Closed-loop Transfer Function  H = P(1 + Q*P);
H{k}= P*(1 + Q*P); 

% Set of stabilizing controllers K= Q / (1+Q*P); 
K{k}= Q/(1+Q*P);

% Transfer Function (square wave response)
y{k}= lsim(H{k},u,t);

if isnan(y{k}) % checks if value is NaN and break 
   fpprintf('error');
   break
end

% Cost Function J(T, Q(Phi))

J(k) = sum((y{k} - ym).^2); % Sum of squared error

if J(k) < max_e 
    fprintf('cost function smaller than 0.01');
    break;
end

% Perturbe the parameter Phi
phi_p = randn(1,length(v_phi(1,:)))*var; % Noise 

% Q(Phi + Phi_p)
Qp = 1;
Ap = 1;
Bp =1;

for i = 1:length(v_phi(k,:))
Ap = Ap*((v_phi(k,i)+phi_p(i))*s^i);
Bp = Bp*(s-alpha(i));
end

Qp= Ap/Bp;

H{k}= P*(1 + Qp*P);

K{k}= Qp/(1+Qp*P);

yp= lsim(H{k},u,t);

% J(T, Q(Phi+Phi_p))
J_p = sum((yp - ym).^2); 

% Adaptive eta

if k>1 && (J(k)-J(k-1)) > (J_rel/J(k-1)) && J(k)> J_th
    eta= eta/1.5;
    fprintf('diminuiu eta ');
end

if k>1 && (J(k)-J(k-1)) > (J_rel_min/J(k-1)) && J(k)< J_th
    eta= eta/1.5;
    fprintf('diminuiu eta ');
end

if eta < min_eta
   eta= min_eta;
end
   
% Update the Parameter
v_phi(k+1,:)= v_phi(k,:) - eta*(J_p - J(k))*phi_p; 

% Detect local minima 

if k>1 && abs(J(k)-J(k-1)) <= tolerance
    count = count +1;
end

if k>2 && abs(J(k)-J(k-2)) <= tolerance
   count = 0;
end

if count > 50
   fprintf('!!! probable local min');
   fprintf('increased var');
   var = var/10;
   count=0;
end


disp(['count= ',num2str(count), ', eta = ', num2str(eta),', i: ', num2str(k),', J = ', num2str(J(k)), ', v_phi = ', num2str(v_phi(k,:))]);

% Plot 
plot(t, ym, 'DisplayName','reference');
hold on
plot(t,y{k}, 'DisplayName','current y');
lgd =legend;
title('Approximation to the Reference (GIF)');
drawnow
hold off
 
% GIF

% Capture the plot as an image 
frame = getframe(f); 
img = frame2im(frame); 
[x,colormap] = rgb2ind(img,256); 

if k == 1 
      imwrite(x,colormap,filename,'gif', 'Loopcount',inf); 
  else 
      imwrite(x,colormap,filename,'gif','WriteMode','append'); 
end 

end

% Closed loop system with final controller 
 yf= lsim(H{k},u,t);


end