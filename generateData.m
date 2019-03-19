% Generation of input data.
% UBC Sailbot
% Nicolas Navarre
% Feb 25 2019
%%
v_rel_in = 20*rand; % relative wind speed from 0 to 20 m/s
w_rel_in = rand*2*pi-pi; %wind angle angle from -pi to pi rad
psi_in = rand*2*pi-pi; % heading angle from -pi to pi rad
phi_in = rand*pi-pi/2; %heel angle from -pi/2 to pi/2
x_in = rand*1000; %(gps) x-position from 0 to 1000 m
dx_in = rand*40-20; %x-velocity from -20 to 20 m/s
y_in = rand*1000; %(gps) y-position from 0 to 1000 m
dy_in = rand*40-20; %y-velocity from -20 to 20 m/s
rangle_in = rand*2*pi/3-pi/3; %rudder angle from -pi/3 to pi/3
roll_in = rand*2*pi-pi;
yaw_in = rand*2*pi-pi;

if w_rel_in > 0
    sangle_in = rand*pi/2;
else
    sangle_in = -rand*pi/2;
end
boat_speed_in = sqrt(dy_in^2 + dx_in^2);


% True wind angle and velocity
v_true = sqrt(v_rel_in^2 + boat_speed_in^2 - 2*v_rel_in*boat_speed_in*cos(w_rel_in));

if w_rel_in > 0 
    w_true = acos((v_rel_in*cos(w_rel_in) - boat_speed_in)/v_true)+psi_in;
else
    w_true = -acos((v_rel_in*cos(w_rel_in) - boat_speed_in)/v_true)+psi_in;
end
%%
save('inputData');