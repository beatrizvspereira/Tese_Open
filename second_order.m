function [T,P] = second_order()
% 2nd order systems

% Ideal Transfer Function (Reference)
% 
% omega =70;
% ksi = 0.3;

% omega=20;
% ksi=0.5;

omega=50;
ksi=0.7;

T = tf(omega^2, [1, 2*ksi*omega, omega^2]); 
%T=tf([1,-2],[1,2,1]);

% Plant

%omega_p = 10; 
%ksi_p = 0.01; % very oscillatory

% omega_p = 10; 
% ksi_p = 0.1;

%faster, less oscillatory 
% omega_p= 30;
% ksi_p=0.5;

%P = tf(omega_p^2, [1, 2*ksi_p*omega_p, omega_p^2])*tf(1,[1,0]);
%P = tf(omega_p^2, [1, 2*ksi_p*omega_p, omega_p^2]);

% non-minimum phase system and open loop unstable
%P= tf([1,-0.01],[1,-2,1])*tf(1,[1,0]);
%P= tf([1,-0.1],[1,0.2])*tf(1,[1,0]);
P= tf([1,-2],[1,4,3])*tf(1,[1,0]);

% non-minimum phase system and open loop stable (marginally stable)
%P= tf([1,-0.001],[1,0.5,0.06])*tf(1,[1,0]);
%P= tf([1,-0.1],[1,0.5,0.06])*tf(1,[1,0]);

% open loop unstable
%P= tf([1,2],[1,-2,1])*tf(1,[1,0]);
%P= tf([1,2],[2,-3,3]);

% Open loop stable, more poles
%P=tf([3,6],[1,3,7,5])*tf(1,[1,0]);

end