% Generation of input data.
% UBC Sailbot
% Nicolas Navarre
% Feb 25 2019
%%
v_rel_in = 20*rand; % relative wind speed from 0 to 20 m/s
w_rel_in = rand*2*pi-pi; %wind angle angle from -pi to pi rad
yaw_in = rand*2*pi-pi; % heading angle from -pi to pi rad
ddyaw_in = rand*pi/2-pi/4; %heading angular acceleration from -pi/4 to pi/4 rad/s^2
roll_in = rand*pi-pi/2; %heel angle from -pi/2 to pi/2
ddroll_in = rand*pi/2-pi/4; %heel angular acceleration rad/s^2
x_in = rand*1000; %(gps) x-position from 0 to 1000 m
dx_in = rand*40-20; %x-velocity from -20 to 20 m/s
y_in = rand*1000; %(gps) y-position from 0 to 1000 m
dy_in = rand*40-20; %y-velocity from -20 to 20 m/s
du_in = rand*10-5; %forward acceleration from -5 to 5 m/s^2
dv_in = rand*10-5; %sideways acceleraton from -5 to 5 m/s^2
rangle_in = rand*2*pi/3-pi/3; %rudder angle from -pi/3 to pi/3
if w_rel_in > 0
    sangle_in = rand*pi/2;
else
    sangle_in = -rand*pi/2;
end

%%
save('inputData');