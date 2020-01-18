function dxdt = simpleBoat_CTS(t, x, u, p1_est)
%PARAMETERS
% u is the input to the system with u = [rudder angle, sail angle]
% x = is the state of the system with x = [x, y, theta, surge, omega]
% 5 states, all are measurbale

%CONSTANTS
p1 = 0.03; % drift coefficent
p2 = 40; % tangential friction kg s^-1
p3 = 6000; % angular friction kg m
p4 = 200; % sail lift kg s^-1
p5 = 1500; % rudder lift kg s^-1
p6 = 0.5; % distance to sail COE [m]
p7 = 0.5; % distance to mast [m]
p8 = 2; % distance to rudder [m]
p9 = 300; % mass of the boat [kg]
p10 = 400; % moment of inertia [kg m^2]
p11 = 0.2; % rudder break coefficient
p1 = p1_est;
%Temp Constants
a_aw = 2; % speed of apparent wind
psi_aw = 3.1415; % angle of apparent wind 
delta_s = u(2); % angle of the sail obtained as input [radians]
delta_r = u(1); % angle of the rudder obtained as input [radians]

a_tw = sqrt(a_aw^2 + x(4)^2 -2*a_aw*x(4)*cos(psi_aw)); % speed of the apparent wind
psi_tw = acos((a_aw*cos(psi_aw)-x(4))/(a_tw)); % angle of apparent wind

g_s =  -p4*a_aw*sin(delta_s - psi_aw); %force on the sail
g_r = -p5*(x(4)^2)*sin(delta_r); % force on the rudder
dxdt = x;
dxdt(1) = x(4)*cos(x(3)) + p1*a_tw*cos(psi_tw);
dxdt(2) = x(4)*sin(x(3)) + p1*a_tw*sin(psi_tw);
dxdt(3) = x(5);
dxdt(4) = (g_s*sin(delta_s) - g_r*p11*sin(delta_r)-p2*(x(4)^2))/p9;
dxdt(5) = (g_s*(p6-p7*cos(delta_s)) -g_r*p8*cos(delta_r) - p3*x(5)*x(4))/p10;