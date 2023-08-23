function [T,P] = higher_order()
% 3rd,4th order systems

% Plant 
% P=tf([3,6],[1,3])*tf(1,[1,0]);

%2nd order
P=tf([3,6],[1,3,7,5]);

% Ideal Transfer Function (Reference)

%4th order
%T=tf([1,3,2],[1,6,8,0,0]); 
T=tf(8,[1,16,70,108,80]);

end