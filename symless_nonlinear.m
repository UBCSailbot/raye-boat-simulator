%declare function
%%
%define inputs
sangle =1;  % sail angle in b-frame (same as delta_s in paper)
rangle = 1; % rudder angle in b-frame (same as delta_r in paper)
v_tw = 1;   % 
alpha_tw =1;

% Note: n-frame is xyz North-East-Down
%       b-frame is xyz Forward-Right-Down

%state inputs
x=1;    % translation in n-frame
y=1;    % translation in n-frame
phi=1;  % angle around x-axis n-frame
psi=1;  % angle around z-axis n-frame (note: z is point down)

% n = [x;  % p.3 left, vector of position+orientation in n-frame
%      y;
%      phi;
%      psi];

surge=1; % forward linear velocity in b-frame (same as u in paper)
sway=1;  % right linear velocity in b-frame (same as v in paper)
roll=1;  % angular velocity around x-axis in b-frame (same as p in paper)
yaw=1;   % angular velocity around z-axis in b-frame (same as r in paper)

%define V p.3 left
vss = [surge;  % vector linear+angular velocity in b-frame
       sway;
       roll;
       yaw];
   
%define J p.3 right
J = [cos(psi) -sin(psi)*cos(phi)  0     0;
     sin(psi) cos(psi)*cos(phi)   0     0;
     0           0                1     0;
     0           0                0  cos(phi)];
 
%define n_dot p.4 left
n_dot = J*vss;  % vector linear+angular velocity in n-frame

%%
% Define constants
m = 1000; %kg mass of boat

I_xx = 100; %kg m^2 moment of inertia
I_zz = 100; %kg m^2
I_xz= 100; %kg m^2

A_s = 12.06384791749; %m^2 area of sail
A_r = 1; %m^2 area of rudder
A_h = 1; %m^2 area of hull
A_k = 1; %m^2 area of keel
x_m = 0.5; %m x-coord of mast in b-frame

x_r = 0.1; %m 
y_r = 0.1; %m
z_r = 0.3; %m
x_s = 1; %m
y_s = 1; %m
z_s = 3; %m
x_h = 1; %m
y_h = 1; %m
z_h = 1; %m
x_k = 1; %m
y_k = 1; %m
z_k = 1; %m
xyz_s = [x_s;y_s;z_s];  % CoE sail
xyz_r = [x_r;y_r;z_r];  % CoE rudder
xyz_h = [x_h;y_h;z_h];  % CoE hull
x_sm = norm(xyz_s);

a_right = 1; % same as a in paper Quatratic righting coeficient
b_right = 1; % same as b in paper Linear righting coefficient

c_heel = 1;  % same as c in paper
d_yaw = 1;   % same as d in paper

X_u_dot = 1; % Added mass phenomena
Y_v_dot = 1; % 

rho_w = 997; %kg/m^3 density of water
rho_a = 1.225; %kg/m^3 density of air

k_drag = 1; %drag const (Not defined in paper)
k_lift= 1; %lift const (Not defined in paper)

waterline_vol = 2; %m^3  This might have to be a function of roll
added_mass = waterline_vol * rho_w /2; % Added mass phenomena
%% Where did this come from? ^^

%% define modular blocks
%define M, p.3 right and p.6 left simplification
M_rb = [m 0   0     0;
        0 m   0     0;
        0 0  I_xx -I_xz;
        0 0 -I_xz I_zz]; %bugbc check if Ixz = 0
M_add = diag(added_mass*ones(1,4));% bugbc double check that this approximation is right p 6 of Paper M_A = -diag{X_u_dot, Y_v_dot, K_p_dot, N_r_dot}, on p.3 it says M_A is stricly positive 
M = M_rb + M_add; 

%define C, p.3 right and p.6 left simplification
C_rb = [0 -m*yaw 0 0;
        m*yaw 0 0 0;
        0 0 0 0;
        0 0 0 0];
C_add = added_mass*[0                     0      0   Y_v_dot*sway;
                    0                     0      0   -X_u_dot*surge;
                    0                     0      0   0;
                 -Y_v_dot*sway    X_u_dot*surge  0   0]; % this is on p.6 of paper 
C = C_add + C_rb;

%define g, p.4 left
g = [0;
     0;
     a_right*phi^2+b_right*phi;
     0];
 
 %% Relative Velocities (VECTORS BABY)
%Sail (n-frame), (b-frame)
% This is not used, it is used in derivation, but finally done as below
% 
% R1 = [cos(-psi), -sin(-psi), 0;
%     sin(-psi), cos(-psi), 0;
%     0, 0, 1];
% R2 = [1, 0, 0;
%     0, cos(-phi), -sin(-phi);
%     0, sin(-phi), cos(-phi)];

% v_tw_n = [v_tw*cos(alpha_tw); v_tw*sin(alpha_tw); 0];
% v_tw_b = R2*R1*v_tw_n;

% p.4 right
v_aw_b = [v_tw*cos(alpha_tw-psi) - surge + yaw*y_s;
          v_tw*sin(alpha_tw-psi)*cos(phi) - sway - yaw*x_s + roll*z_s;
          0];
v_aw = norm(v_aw_b);
alpha_aw = atan2(v_aw_b(2),-v_aw_b(1)); % p.4 the negative is correct
alpha_s = alpha_aw - sangle;

%Rudder (b-frame) p.5 left
v_ar_b = [-surge + yaw*y_r;
          -sway - yaw*x_r + roll*z_r; 
          0];
v_ar = norm(v_ar_b);
alpha_ar = atan2(v_ar_b(2),-v_ar_b(1)); % -ve b/c North-East-Down frame
alpha_r = alpha_ar - rangle;

%Keel (b-frame) p.5 left
v_ak_b = [-surge + yaw*y_k;
          -sway - yaw*x_k + roll*z_k;
          0];
v_ak = norm(v_ak_b);
alpha_ak = atan2(v_ak_b(2),-v_ak_b(1));

%Hull (n-frame) p.5 right
v_ah_n = [-surge + yaw*y_h;
          sec(phi)*(-sway - yaw*x_h + roll*z_h);
          0];
v_ah = norm(v_ah_n);
alpha_ah = atan2(v_ah_n(2),-v_ah_n(1));

 %define D
 
 % bugbc! missing D_h is wrong p.5 right, Frh unknown
 D_h = [Frh(v_ah)*cos(alpha_ah);
     -Frh(v_ah)*sin(alpha_ah)*cos(phi);
     (-Frh(v_ah)*sin(alpha_ah)*cos(phi))*abs(z_h);
     Frh(v_ah)*sin(alpha_ah)*cos(phi)*abs(x_h);
 ];

liftk=lift(rho_w,A_h,v_ak,alpha_ak);
dragk=drag(rho_w,A_h,v_ak,alpha_ak);

% p.5 left
 D_k = [-liftk*sin(alpha_ak) + dragk*cos(alpha_ak);
     -liftk*cos(alpha_ak) - dragk*sin(alpha_ak);
     (-liftk*cos(alpha_ak) - dragk*sin(alpha_ak))*abs(z_k);
     (liftk*cos(alpha_ak) + dragk*sin(alpha_ak))*abs(x_k);
 ]

% p.4 left
phi_dot = roll;
D_heel = [0;
    0;
    c_heel*phi_dot*abs(phi_dot);  % note: phi_dot = n_dot(3) = roll  , as shown by n_dot=J*vss
    0];

% p.4 left
psi_dot = yaw*cos(phi);
D_yaw = [0;
    0;
    0; 
    d_yaw*psi_dot*abs(psi_dot)*cos(phi)]; % note: psi_dot = n_dot(4) = yaw*cos(phi), as shown by n_dot=J*vss

D = D_heel + D_h + D_yaw + D_k;

%% define T
% p.5 left
lift_s = lift(rho_a,A_s,v_aw,alpha_s);
drag_s = drag(rho_a,A_s,v_aw,alpha_s);
T_s = [lift_s*sin(alpha_aw) - drag_s*cos(alpha_aw);
    lift_s*cos(alpha_aw) + drag_s*sin(alpha_aw);
    (lift_s*cos(alpha_aw) + drag_s*sin(alpha_aw))*abs(z_s);
    -(lift_s*sin(alpha_aw) - drag_s*cos(alpha_aw))*x_sm*sin(sangle) + (lift_s*cos(alpha_aw) + drag_s*sin(alpha_aw))*(x_m-x_sm*cos(sangle))];

% Rudder Forces p.6 left simplification
drag_r = drag(rho_w,A_r,v_ar,alpha_r);
lift_r = lift(rho_w,A_r,v_ar,alpha_r);
T_r = [-drag_r;
       lift_r;
       lift_r*abs(z_r);
       -lift_r*abs(x_r)];
                 
T = T_s + T_r;

% Righting Moment

v_dot = -inv(M)*C*vss-inv(M)*D-inv(M)*g+inv(M)*T;

%return [n_dot,v_dot]





