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
rCoE_x = 0.1; %m
rCoE_y = 0.1; %m
rCoE_z = 0.3; %m
sCoE_x = 1; %m
sCoE_y = 1; %m
sCoE_z = 3; %m
hCoE_x = 1; %m
hCoE_y = 1; %m
hCoE_z = 1; %m
kCoE_x = 1; %m
kCoE_y = 1; %m
kCoE_z = 1; %m
sCoE = [sCoE_x;sCoE_y;sCoE_z];
rCoE = [rCoE_x;rCoE_y;rCoE_z];
hCoE = [hCoE_x;hCoE_y;rCoE_z];
x_sm = norm(sCoE);
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

zkeel =1 %center of effort bugbc
xkeel =1 %center of effort

%% define modular blocks
%define M
M_rb = [m 0 0 0;
        0 m 0 0;
        0 0 I_xx -I_xz;
        0 0 -I_xz I_zz]; %bugbc Ixz used to be 0
M_add = diag(added_mass*ones(1,4));% bugbc comeback to added mass
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

v_aw_b = [v_tw*cos(alpha_tw-psi)-surge+yaw*sCoE_y;
    v_tw*sin(alpha_tw-psi)*cos(phi)-sway+yaw*sCoE_x+roll*sCoE_z;
    0];

v_aw = norm(v_aw_b);
alpha_aw = atan2(v_aw_b(2),-v_aw_b(1));
alpha_s = alpha_aw - sangle;

%Rudder (b-frame)
v_ar_b = [-surge + yaw*rCoE_y; -sway - yaw*rCoE_x + roll*rCoE_z; 0];
v_ar = norm(v_ar_b);
alpha_ar = atan2(v_ar_b(2),-v_ar_b(1));
alpha_r = alpha_ar - rangle;

%Keel (b-frame)
v_ak_b = [-surge + yaw*kCoE_y; -sway - yaw*kCoE_x + roll*kCoE_z; 0];
v_ak = norm(v_ak_b);
alpha_ak = atan2(v_ak_b(2),-v_ak_b(1));

%Hull (n-frame)
v_ah_n = [-surge + yaw*hCoE_y; sec(phi)*(-sway - yaw*hCoE_x + roll*hCoE_z); 0];
v_ah = norm(v_ah_n);
alpha_ah = atan2(v_ah_n(2),-v_ah_n(1));



 %define D
 
 % bugbc! missing D_h is wrong
 D_h = [drag(rho_w,A_h,v_ah,alpha_ah)*cos(alpha_ah);
    -drag(rho_w,A_h,v_ah,alpha_ah)*sin(alpha_ah)*cos(phi);
    (-drag(rho_w,A_h,v_ah,alpha_ah)*sin(alpha_ah)*cos(phi))*abs(hCoE_z);
    (drag(rho_w,A_h,v_ah,alpha_ah)*sin(alpha_ah)*cos(phi))*abs(hCoE_x)];

liftk=lift(rho_w,A_h,v_ak,alpha_ak);
dragk=drag(rho_w,A_h,v_ak,alpha_ak);

 D_k = [-liftk*sin(alpha_ak)+dragk*cos(alpha_ak);
     -liftk*cos(alpha_ak)-dragk*sin(alpha_ak);
     (-liftk*cos(alpha_ak)-dragk*sin(alpha_ak))*abs(zkeel);
     (liftk*cos(alpha_ak)+dragk*sin(alpha_ak))*abs(xkeel);
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
    (lift(rho_a,A_s,v_aw,alpha_s)*cos(alpha_aw)+drag(rho_a,A_s,v_aw,alpha_s)*sin(alpha_aw))*abs(sCoE_z);
    -(lift(rho_a,A_s,v_aw,alpha_s)*sin(alpha_aw)-drag(rho_a,A_s,v_aw,alpha_s)*cos(alpha_aw))*x_sm*sin(sangle) + (lift(rho_a,A_s,v_aw,alpha_s)*cos(alpha_aw)+drag(rho_a,A_s,v_aw,alpha_s)*sin(alpha_aw))*(x_m-x_sm*cos(sangle))];
% Rudder Forces
T_r = [-drag(rho_w,A_r,v_ah,alpha_r);
    lift(rho_w,A_r,v_ar,alpha_r);
    lift(rho_w,A_r,v_ar,alpha_r)*abs(rCoE_z);
    -lift(rho_w,A_r,v_ar,alpha_r)*abs(rCoE_x)];
                 
T = T_s + T_r;

% Righting Moment

v_dot = -inv(M)*C*vss-inv(M)*D-inv(M)*g+inv(M)*T;

%return [n_dot,v_dot]






