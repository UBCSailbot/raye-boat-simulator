% Define constants

%% Mass and inertia
m = 1000; % kg, mass of boat

I_xx = 100; % kg*m^2, moment of inertia
I_zz = 100; % kg*m^2
I_xz = 100; % kg*m^2

M_rb = [m 0   0     0;
        0 m   0     0;
        0 0  I_xx -I_xz;
        0 0 -I_xz I_zz]; %bugbc check if Ixz = 0
M_add = diag(added_mass*ones(1,4)); % bugbc double check that this approximation is right: p.6 left M_A = -diag{X_u_dot, Y_v_dot, K_p_dot, N_r_dot}, on p.3 it says M_A is stricly positive 
M = M_rb + M_add; 

%% Areas
A_s = 12.06384791749; % m^2, area of sail
A_r = 1; % m^2, area of rudder
A_h = 1; % m^2, area of hull
A_k = 1; % m^2, area of keel TODO: figure out if use A_h or A_k for keel lift/drag, only use one, guess A_k for now, maybe A_h in F_rh

%% Mast location
x_m = 0.5; % m, x-coord of mast in b-frame

%% Center of Efforts
x_r = 0.1; % m 
y_r = 0.1; % m
z_r = 0.3; % m
x_s = 1; % m
y_s = 1; % m
z_s = 3; % m
x_h = 1; % m
y_h = 1; % m
z_h = 1; % m
x_k = 1; % m
y_k = 1; % m
z_k = 1; % m
xyz_s = [x_s;y_s;z_s];  % CoE sail
xyz_r = [x_r;y_r;z_r];  % CoE rudder
xyz_h = [x_h;y_h;z_h];  % CoE hull
x_sm = norm(xyz_s);

%% Force constants
a_right = 1; % same as a in paper Quatratic righting coeficient
b_right = 1; % same as b in paper Linear righting coefficient

c_heel = 1;  % same as c in paper
d_yaw = 1;   % same as d in paper

%% Lift and Drag Constants
k_drag = 1; % drag const (Not defined in paper)
k_lift= 1; % lift const (Not defined in paper)
k_rh = 1; % Frh const (Not defined in paper)

%% Added mass
X_u_dot = 1; % Added mass phenomena
Y_v_dot = 1; % 
rho_w = 997; % kg/m^3, density of water
rho_a = 1.225; % kg/m^3, density of air
waterline_vol = 2; % m^3  This might have to be a function of roll
added_mass = waterline_vol * rho_w /2; % Added mass phenomena <-not sure where this is from