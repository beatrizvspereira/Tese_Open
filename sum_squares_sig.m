function [u,t] = sum_squares_sig()

% % Signal 1
% t=(0:0.01:10);
% 
% u_1= square(t,50);
% u_2= square(t-4,20);
% u_3= square(t+9,5);
% u= u_1+2*(u_2)+(u_3);
% 
% u=u';

tau0=15;
%Tf = 100;
Tf=60;
%Ts=0.06250;
%Ts=0.03125;
Ts=0.001;

% % Signal 2
t=(0:Ts:Tf);
%t=(0:0.001:100);

u0 = gensig("square",tau0,Tf, Ts);

tau1=40;
u1 = gensig("square",tau1,Tf, Ts);

u= (2*u0+u1)*(1e-3);

% Signal 3
% tau2=20;
% u2 = gensig("square",tau2,Tf, Ts);
% 
% u = 2*u0+u1+2*u2;


end