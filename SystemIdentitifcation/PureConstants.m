%% Define constants
% Information is based on
% https://ubcsailbot.atlassian.net/wiki/spaces/ADA2/pages/72450411/Parameters+Used+In+Control+Software+s+Boat+Model
% as of Oct 15, 2019
% Not complete on Oct 15, 2019. Refer to link above for correct values

%% Mass and inertia
m = 350; % kg, mass of boat (From link)

I_xx = 50000; % kg*m^2, moment of inertia (got from paper)
I_zz = 10000; % kg*m^2 (got from paper)
I_xz = 500; % kg*m^2 (got from paper)

%% Areas
A_s = 8.99 + 3.07; % m^2, area of sail (Sail + Jib) (From link)
A_r = 0.129; % m^2, area of rudder (From link)
%A_h = 1; % m^2, area of hull (note needed because paper doesn't use this
%parameter)
A_k = 0.560525; % m^2, area of keel (From link) TODO: figure out if use A_h or A_k for keel lift/drag, only use one, guess A_k for now, maybe A_h in F_rh

%% Mast location
x_m = 0.1; % m, x-coord of mast in b-frame

%% Center of Efforts
x_r = -5.5/2; % m (from link CAD model)
y_r = 0; % m (from link)
z_r = -0.5; % m (from link CAD model)
x_s = -1; % m (estimate from link)
y_s = 0; % m (from link)
z_s = -5; % m (estimate from link)
x_h = 0; % m 
y_h = 0; % m
z_h = 0; % m
x_k = 0; % m (estimate by Bruce)
y_k = 0; % m (estimate by Bruce)
z_k = 1.5; % m (estimate from CAD model)

%% Force constants
a_right = -5.9; % same as a in paper Quatratic righting coeficient
b_right = 8160; % same as b in paper Linear righting coefficient

c_heel = 120000;  % same as c in paper
d_yaw = 20000;   % same as d in paper

%% Lift and Drag Constants
k_drag = 0.2; % drag const (Not defined in paper)
k_lift= 0.2; % lift const (Not defined in paper)
k_rh = 50; % Frh const (estimate by Bruce)

%% Added mass
X_u_dot = 1; % Added mass phenomena
Y_v_dot = 1; % 
rho_w = 997; % kg/m^3, density of water
rho_a = 1.225; % kg/m^3, density of air
waterline_vol = 2; % m^3  This might have to be a function of roll
