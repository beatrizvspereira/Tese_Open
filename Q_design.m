function [v_phi,y,J,k, K,yf,H,Q_f,J_min,k_min,phi_min,v_eta,step_size]=Q_design(P,N,phi_initial,ym, y_ref, max_e,eta,u,t,alpha, s0_r0,theta_reference,c1,c2,c_inv)

v_phi= zeros(N, length(phi_initial));
v_phi(1,:)= phi_initial;
v_eta=zeros(1,N);
v_eta(1,:)= eta;
step_size=zeros(1,N);

% Plant
s=tf('s');

% Controller
[num_sys,den_sys] = tfdata(P);
[num,den] = tfdata(s0_r0);

J= zeros(1,N);

if c1>0
f = figure;
filename = 'follow_ref.gif';
end 

% Adjustable parameters

%tolerance = 0.01;
J_threshold=1e-5;
min_eta = 0.001; % min. value accepted for eta
J_rel = 100; % percentage increase (uma ordem de grandeza)
J_rel_2 = 10; 
flag_it=0;

J_min=10; 
J_max=1e-10;


k_min=0;
k_max=0;

phi_min=0;
phi_max= 0;

grad_cont=0;
count_inv=0;
%count=0;
%count_2=1;
%save_J=zeros(1,N/100);

% Preallocate Memory for Cell Array
Q_f{N} = [];
K{N} = [];
theta_f{N} = [];
H{N} = [];
y{N} = [];
theta_f_p{N} = [];

% X_m= zeros(3,3);
% Y_m= zeros(3,1);

for k = 1:N

% Matrix Q: Finite Dimension Approximation 
Q = 0;

for i = 1:length(v_phi(k,:))
Q = Q + v_phi(k,i)*((alpha/(s+alpha))^(i-1));
end

Q=minreal(Q);
[numQ,denQ] = tfdata(Q);
Q_f{k}=Q;

% Set of stabilizing controllers K= Q / (1+Q*P) or  K= (Sº+Q*A) / (Rº - Q*B); 
K{k}= tf((conv(denQ{1},num{1})+conv(numQ{1},den_sys{1})),(conv(denQ{1},den{1})-conv(numQ{1},num_sys{1})));

SR=K{k};
assignin('base','SR',SR);

if c2>0
out=sim('fminunc_rocketmodel',60);
theta_f{k}=out.theta;
end

% Closed-loop Transfer Function  H = P(1 + Q*P) or H = P / (1+P*K)
%H{k}= minreal(P/(1 + P*K{k})); 
H{k}=minreal((K{k}*P)/(1+K{k}*P));

% Transfer Function (square wave response)
y{k}= lsim(H{k},u,t);

if isnan(y{k}) % checks if value is NaN and break 
   fpprintf('error');
   break
end

% Cost Function J(T, Q(Phi))

if c1>0
J(k) = c1*sum((y{k} - ym).^2); % Sum of squared error
end

if c2>0
    J(k)= c2*sum((theta_reference - theta_f{k}).^2);
end

if c_inv>0
y_2= step(H{k},t); % Step Response of CLS (K_controller)
J_inv= sign(y_2.*y_ref); % if sign is negative, penalize
count_inv = sum(J_inv == -1);
J(k) = c1*sum((y{k} - ym).^2)+ c_inv*count_inv; % Sum of squared error
end

if J(k) < max_e 
    fprintf(['cost function smaller than ', num2str(max_e)]);
    break;
end

% Perturbe the parameter Phi
phi_p = randn(1,length(v_phi(1,:))); % Noise 
%phi_p = randn(1,length(v_phi(1,:)))*(1e-4); % Noise 
%phi_p = randn(1,length(v_phi(1,:)))*10; % Noise 

% Q(Phi + Phi_p)
Qp=0;

for i = 1:length(v_phi(k,:))
Qp = Qp + (v_phi(k,i)+phi_p(i))*((alpha/(s+alpha))^(i-1));
end
Qp= minreal(Qp);
[numQp,denQp] = tfdata(Qp);

K{k}= tf((conv(denQp{1},num{1})+conv(numQp{1},den_sys{1})),(conv(denQp{1},den{1})-conv(numQp{1},num_sys{1})));
SR=K{k};
assignin('base','SR',SR);

if c2>0
out_2=sim('fminunc_rocketmodel',60);
theta_f_p{k}=out_2.theta; 
end 

%H{k}= minreal(P/(1 + P*K{k}));
H{k}=minreal((K{k}*P)/(1+K{k}*P));

yp= lsim(H{k},u,t);

% J(T, Q(Phi+Phi_p))
J_p = c1*sum((yp - ym).^2)+ c2*sum((theta_reference - theta_f_p{k}).^2);

if c_inv>0
y_2p= step(H{k},t); % Step Response of CLS (K_controller)
J_inv= sign(y_2p.*y_ref); % if sign is negative, penalize
count_inv = sum(J_inv == -1);
J_p = c1*sum((yp - ym).^2)+ c_inv*count_inv; % Sum of squared error
end

% Adaptive eta

% if k>1 && (J(k)-J(k-1)) > (J_rel/J(k-1))
%     eta= eta/2;
%     flag_it=1;
%     fprintf('eta diminui');
% end

if k>1
percentage_increase = ((J(k) - J(k-1)) / J(k-1)) * 100;
end 

if k>1 && percentage_increase >= J_rel && J(k) <= J_threshold
    v_eta(k)= v_eta(k)/3;
    flag_it=1;
    rand_factor= rand(); 
    fprintf(' eta reduced  ');
end

if k>1 && percentage_increase >= J_rel_2 && J(k) >= J_threshold
    v_eta(k)= v_eta(k)/3;
    flag_it=1;
    rand_factor= rand(); 
    fprintf('  Eta reduce/ above threshold  ');
end

if flag_it == 1 && rand_factor >= 0.99 % 1% of the times the eta does not change
    flag_it=0;
    fprintf('  Random factor above 0.99  ')
end

if k > (N-(N*0.05))  
    v_eta(k)= v_eta(k)/(N-(N-(N*0.05)));
    %eta=eta/2;
    fprintf('  Eta reduced (last 5perct. it.).  ');
end

if v_eta(k) < min_eta
   v_eta(k)= min_eta;
end

if k>1 && J(k) < J_min 
    J_min = J(k);
    k_min= k;
    phi_min= v_phi(k,:);
end
   
if k>1 && J(k) > J_max 
    J_max = J(k);
    k_max= k;
    phi_max= v_phi(k,:);
end

eta_norm= (v_eta(k)/J(k));
%eta_norm= v_eta(k);

% Update the Parameter
if flag_it == 0
v_phi(k+1,:)= v_phi(k,:) - (eta_norm)*(J_p - J(k))*phi_p; 
end

%update eta
if k<N
v_eta(k+1)=v_eta(k);
end 

% Method 1
if flag_it==1

% Change actual J to previous value
v_phi(k,:)=v_phi(k-1,:);
J(k)= J(k-1);

v_phi(k+1,:)= v_phi(k-1,:); % Next phi is equal to the previous phi
fprintf('  Next phi changed to previous phi value.  ')

flag_it=0;
end

if c2>0
step_size(k)=(eta_norm)*(J_p - J(k))*phi_p;

disp(['i: ', num2str(k),', eta = ', num2str(v_eta(k)), ', eta/J = ', num2str(eta_norm), ' ,J = ', num2str(J(k)),' , v_phi = ', num2str(v_phi(k,:)),', step= ', num2str(step_size(k)), ' ,phi_p= ', num2str(phi_p),' ,J_min= ', num2str(J_min)]);
else
disp(['i: ', num2str(k),', eta = ', num2str(v_eta(k)), ', eta/J = ', num2str(eta_norm), ' ,J = ', num2str(J(k)),' , v_phi = ', num2str(v_phi(k,:)), 'phi_p= ', num2str(phi_p),' ,J_min= ', num2str(J_min)]);
end 

%##########################

% Conjugate Gradient Method
grad_cont=grad_cont+1;

if grad_cont==3 
v_phi(k+1,:)= v_phi(k,:)+ (v_phi(k,:)-v_phi(k-2,:));
grad_cont=0;
end

%##########################

% if k>1 && J(k) < J(k-1)
%     eta= 2*eta;
%     fprintf('double eta');
% end

% if k>1 && J(k) > J(k-1)
%    X_m(3,:)= [eta^2, eta, 1]; 
%    X_m(2,:)= [(eta/2)^2, (eta/2), 1]; 
%    X_m(1,:)= [(eta/2)^2, (eta/4), 1]; 
%    Y_m= [J(k-2);J(k-1);J(k)];
%    coef=X_m \ Y_m;
%    eta_line=-coef(2)/(2*coef(1));
% end

if v_phi(k+1,:)<(-1050)
   v_phi(k+1,:)=-1020;
end

% % Save Cost Array matfile
% 
% count=count+1;
% 
% if count == 100
%     save_J(count_2)= J(k);
%     count=0;
%     count_2=count_2+1;
% end 
    
% GIF plot

if c1>0
% Plot 
plot(t, ym, 'DisplayName','reference');
hold on
plot(t,y{k}, 'DisplayName','current y');
lgd =legend;
title('Approximation to the Reference (GIF)');
drawnow
hold off

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

% Save cost

%j_file = matfile('rlcost_27.mat','Writable',true);
save('rlcost_47.mat','J','J_min', 'J_max');
save('phi_47', 'phi_min', 'phi_max', 'k_min','k_max');

% Closed loop system with final controller 
 yf= lsim(H{k},u,t);

end