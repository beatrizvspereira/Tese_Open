function [u,t] = square_sig()

tau=40;
Tf = 60;
% Ts=0.1;
% Ts= 0.06250;
Ts= 1/1000;

[u0,t] = gensig("square",tau,Tf, Ts);
u = (2*u0-1)*(1e-3);

end