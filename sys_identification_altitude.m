function [A2,B2,C2,D2,beta] =sys_identification_altitude()

%System Identification (Altitude) 

load('id_data_altitude.mat')

Ts=mean(diff(ts));

Nx=0; % System order

data_fe=iddata(ys3,us3,Ts);

data_fe.InputName  = {'Delta Thrust'};
data_fe.OutputName = {'d2z'};

sys_alt_est=ssest(data_fe,Nx)

figure;
compare(data_fe,sys_alt_est,'--');

beta=sys_alt_est.D;

id2_tf=tf(beta,[1 0 0]);

[A2,B2,C2,D2]=tf2ss(id2_tf.numerator{1},id2_tf.denominator{1});
sys_alt=ss(A2,B2,C2,D2);

save('ss_model_altitude.mat','A2','B2','C2','D2');
end