function [T,P,flag_rocket] = rocket_system()

flag_rocket=1;

load('ss_model.mat');
[num,den] = ss2tf(A,B,C,D);
P= tf(num,den);

%Underdamped System
%omega= 35.36; % natural frequency
%ksi=0.71; % dampling factor

omega=20;
ksi = 0.5; % dampling factor

T = tf(omega^2, [1, 2*ksi*omega, omega^2]); 

end