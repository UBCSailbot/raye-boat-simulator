%declare function
%%
%define inputs
sangle =1;
rangle = 1;
v_tw = 1;
alpha_tw =1;

%state inputs
x=1;
phi=1;
psi=1;
roll=1;
surge=1;
sway=1;
yaw=1;
%%
% Define constants
m = 1000; %kg
I_xx = 100; %kg m^2
I_zz = 100; %kg m^2
I_xz= 100; %kg m^2

A_s = 12.06384791749; %m^2
A_r = 1; %m^2
A_h = 1; %m^2
A_k = 1; %m^2
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
xyz_s = [x_s;y_s;z_s];
xyz_r = [x_r;y_r;z_r];
xyz_h = [x_h;y_h;z_r];
x_sm = norm(xyz_s);
heel_const = 1; %testing?
a_right = 1; % Quatratic righting coeficient
b_right = 1; % Linear righting coefficient
X_u_dot = 1;
Y_v_dot = 1;
c_heel = 1;
d_yaw = 1;

rho_w = 997; %kg/m^3
rho_a = 1.225; %kg/m^3
k1 = 1; %lift/drag const (This can be tuned)
waterline_vol = 2; %m^3  This might have to be a function of roll
added_mass = waterline_vol * rho_w /2; %

z_k =1 %center of effort bugbc
x_k =1 %center of effort
z_h = 1
x_h = 1

%% define modular blocks
%define M
M_rb = [m 0 0 0;
        0 m 0 0;
        0 0 I_xx -I_xz;
        0 0 -I_xz I_zz]; %bugbc check if Ixz = 0
M_add = diag(added_mass*ones(1,4));% bugbc double check that this approximation is right
M = M_rb + M_add; 

%define C
C_rb = [0 -m*yaw 0 0;
        m*yaw 0 0 0;
        0 0 0 0;
        0 0 0 0];
C_add = added_mass*[0 0 0 Y_v_dot*sway;
                    0 0 0 -X_u_dot*surge;
                    0 0 0 0;
                    -Y_v_dot*sway X_u_dot*surge 0 0];
C = C_add + C_rb;

%define J
J = [cos(psi) -sin(psi)*cos(phi) 0 0;
     sin(psi) cos(psi)*cos(phi) 0 0;
     0 0 1 0;
     0 0 0 cos(phi)];
 
%define V
vss = [surge;sway;roll;yaw];

%define n_dot
n_dot = J*vss;

%define g
g = [0;
     0;
     a_right*phi^2+b_right*phi;
     0];
 %% Relative Velocities (VECTORS BABY)
%Sail (n-frame), (b-frame)
R1 = [cos(-psi), -sin(-psi), 0;
    sin(-psi), cos(-psi), 0;
    0, 0, 1];
R2 = [1, 0, 0;
    0, cos(-phi), -sin(-phi);
    0, sin(-phi), cos(-phi)];

v_tw_n = [v_tw*cos(alpha_tw); v_tw*sin(alpha_tw); 0];
v_tw_b = R2*R1*v_tw_n;

v_aw_b = [v_tw*cos(alpha_tw-psi)-surge+yaw*y_s;
    v_tw*sin(alpha_tw-psi)*cos(phi)-sway+yaw*x_s+roll*z_s;
    0];

v_aw = norm(v_aw_b);
alpha_aw = atan2(v_aw_b(2),-v_aw_b(1));
alpha_s = alpha_aw - sangle;

%Rudder (b-frame)
v_ar_b = [-surge + yaw*y_r; -sway - yaw*x_r + roll*z_r; 0];
v_ar = norm(v_ar_b);
alpha_ar = atan2(v_ar_b(2),-v_ar_b(1));
alpha_r = alpha_ar - rangle;

%Keel (b-frame)
v_ak_b = [-surge + yaw*y_k; -sway - yaw*x_k + roll*z_k; 0];
v_ak = norm(v_ak_b);
alpha_ak = atan2(v_ak_b(2),-v_ak_b(1));

%Hull (n-frame)
v_ah_n = [-surge + yaw*y_h; sec(phi)*(-sway - yaw*x_h + roll*z_h); 0];
v_ah = norm(v_ah_n);
alpha_ah = atan2(v_ah_n(2),-v_ah_n(1));



 %define D
 
 % bugbc! missing D_h is wrong
 D_h = [Frh(v_ah)*cos(alpha_ah);
     -Frh(v_ah)*sin(alpha_ah)*cos(phi);
     (-Frh(v_ah)*sin(alpha_ah)*cos(phi))*abs(z_h);
     Frh(v_ah)*sin(alpha_ah)*cos(phi)*abs(x_h);
 ];

liftk=lift(rho_w,A_h,v_ak,alpha_ak);
dragk=drag(rho_w,A_h,v_ak,alpha_ak);

 D_k = [-liftk*sin(alpha_ak)+dragk*cos(alpha_ak);
     -liftk*cos(alpha_ak)-dragk*sin(alpha_ak);
     (-liftk*cos(alpha_ak)-dragk*sin(alpha_ak))*abs(z_k);
     (liftk*cos(alpha_ak)+dragk*sin(alpha_ak))*abs(x_k);
 ]

D_heel = [0;
    0;
    c_heel*roll*abs(roll);
    0];

D_yaw = [0;
    0;
    0;
    d_yaw*yaw*abs(yaw)*cos(phi)];

D = D_heel + D_h + D_yaw+D_k;

%% define T

T_s = [lift(rho_a,A_s,v_aw,alpha_s)*sin(alpha_aw)-drag(rho_a,A_s,v_aw,alpha_s)*cos(alpha_aw);
    lift(rho_a,A_s,v_aw,alpha_s)*cos(alpha_aw)+drag(rho_a,A_s,v_aw,alpha_s)*sin(alpha_aw);
    (lift(rho_a,A_s,v_aw,alpha_s)*cos(alpha_aw)+drag(rho_a,A_s,v_aw,alpha_s)*sin(alpha_aw))*abs(z_s);
    -(lift(rho_a,A_s,v_aw,alpha_s)*sin(alpha_aw)-drag(rho_a,A_s,v_aw,alpha_s)*cos(alpha_aw))*x_sm*sin(sangle) + (lift(rho_a,A_s,v_aw,alpha_s)*cos(alpha_aw)+drag(rho_a,A_s,v_aw,alpha_s)*sin(alpha_aw))*(x_m-x_sm*cos(sangle))];
% Rudder Forces
T_r = [-drag(rho_w,A_r,v_ar,alpha_r);
    lift(rho_w,A_r,v_ar,alpha_r);
    lift(rho_w,A_r,v_ar,alpha_r)*abs(z_r);
    -lift(rho_w,A_r,v_ar,alpha_r)*abs(x_r)];
                 
T = T_s + T_r;

% Righting Moment

v_dot = -inv(M)*C*vss-inv(M)*D-inv(M)*g+inv(M)*T;

%return [n_dot,v_dot]





