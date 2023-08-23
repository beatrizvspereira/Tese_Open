% Constants
l1=60; % longitudinal length between the COG of the rocket and Fe
l2=10; % longitudinal length between the COG of the rocket and Fr,Fl
ln=1; % lentgh of the nozzle
m=549054; % sum of the rocket's dry mass and fuel mass;
g=9.8; %gravity

% Initial Positions
z_0=4000; % vertical position

% Equilibrium
thrust_eq=m*g;
gimbal_eq=0;

load('ss_model');

T_stop=40;
fs=1000;
Ts=1/fs;
t_signal=(0:Ts:T_stop)';
phi=(1e-6)*square(t_signal,25);
%phi=wgn(1000,1,-30); % white guassian noise with power -30dbW (1e-6 watt)

Fe=ones(length(phi),1)*(2*10^6); %descend

delta_thrust= Fe-thrust_eq;
delta_angle= phi-gimbal_eq;

gimbal=gimbal_eq+delta_angle;
thrust=thrust_eq+delta_thrust; 

t=linspace(0,40,length(gimbal));
t=t';

%% Simulation

lin_rocketmodel;
sim('lin_rocketmodel');

%% Controller

% Controlability
Co =  ctrb(A,B);
if rank(Co) ~= size(A)
    disp('Not Controlable');
end

% Observability
Ob = obsv(A,C);
if rank(Ob) ~= size(A)
     disp('Not observable');
end

% Regulator Design (LQR)

Q=C'*C; %Weight Matrix for x
R=1; %Weight for the input variable

% optimal gain matrix K 
K = lqr(A,B,Q,R);
poles = eig(A-B*K);
