 function [A,B,C,D,alpha] =sys_identification_attitude()

%System Identification (Atitude) 

% load('id_data.mat')
load('id_data_atitude.mat')

Ts=mean(diff(ts));

Nx=0; % System order

data=iddata(ys2,us2,Ts);

% data.InputName  = {'Delta Main Thrust';'Delta Gimbal Angle'};
% data.OutputName = {'d2x';'d2z';'d2theta'};

data.InputName  = {'Delta Gimbal Angle'};
data.OutputName = {'d2theta'};

sys_est=ssest(data,Nx)

figure;
compare(data,sys_est,'--');

alpha= sys_est.D;

id_tf=tf(alpha,[1 0 0]);

[A,B,C,D]=tf2ss(id_tf.numerator{1},id_tf.denominator{1});
sys=ss(A,B,C,D);

save('ss_model.mat','A','B','C','D');
end