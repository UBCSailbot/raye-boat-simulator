%% Update input Data

load('inputData');
load('simuData');

time = simuData(1,:);
x_out = simuData(2,:);
y_out = simuData(3,:);
phi_out = simuData(4,:);
psi_out = simuData(5,:);

dxdt = diff(x_out)./diff(time);
dydt = diff(y_out)./diff(time);
dphidt = diff(phi_out)./diff(time);
dpsidt = diff(psi_out)./diff(time);

x_in = x_in + x_out(end); %New x-coordinate, m
y_in = y_in + y_out(end); %New y-coordinate, m
phi_in = phi_in + phi_out(end); %New roll angle radians
psi_in = psi_in + psi_out(end); %New yaw angle (pointing), radians

dx_in = dx_in + dxdt(end); %New x-velocity (n-frame), m/s
dy_in = dy_in + dydt(end); %New y-velocity (n-frame), m/s
roll_in = roll_in + dphidt(end); %New roll (n-frame), rad/s
yaw_in = yaw_in + dpsidt(end); %New yaw (n-frame), rad/s


save('inputData','x_in','y_in','phi_in','psi_in','dx_in','dy_in','roll_in','yaw_in','v_tw','alpha_tw','sangle_in','rangle_in');
