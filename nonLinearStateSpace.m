function dxdt = nonLinearStateSpace (xInput,u)
%% define inputs
Constants_Oct_15_2019;

sangle = u(1);   % sail angle in b-frame (same as delta_s in paper)
rangle = u(2);   % rudder angle in b-frame (same as delta_r in paper)
v_tw = u(3);     % speed of wind in n-frame
alpha_tw = u(4); % angle of wind in n-frame

% Note: n-frame is xyz North-East-Down
%       b-frame is xyz Forward-Right-Down

%% define state space
x = xInput(1);    % m, position in n-frame
y = xInput(2);    % m, position in n-frame
phi = xInput(3);  % rad, angle around x-axis n-frame
psi = xInput(4);  % rad, angle around z-axis n-frame (note: z is pointed down)

% For reference: p.3 left, vector of position+orientation in n-frame
% n = [x;
%      y;
%      phi;
%      psi];

surge=xInput(5); % m/s, linear velocity forward in b-frame (same as u in paper)
sway=xInput(6);  % m/s, linear velocity right in b-frame (same as v in paper)
roll=xInput(7);  % rad/s, angular velocity around x-axis in b-frame (same as p in paper)
yaw=xInput(8);   % rad/s, angular velocity around z-axis in b-frame (same as r in paper)

% p.3 left, vector linear+angular velocity in b-frame
vss = [surge; 
       sway;
       roll;
       yaw];
%    
% p.3 right
J = [cos(psi) -sin(psi)*cos(phi)  0     0;
     sin(psi) cos(psi)*cos(phi)   0     0;
     0           0                1     0;
     0           0                0  cos(phi)];
%  
% % p.4 left, vector linear+angular velocity in n-frame
n_dot = J*vss;

%% define C
% p.3 right and p.6 left simplification
C_rb = [0 -m*yaw 0 0;
        m*yaw 0 0 0;
        0 0 0 0;
        0 0 0 0];
C_add = added_mass*[0                     0      0   Y_v_dot*sway;
                    0                     0      0   -X_u_dot*surge;
                    0                     0      0   0;
                 -Y_v_dot*sway    X_u_dot*surge  0   0]; % this is on p.6 of paper 
C = C_add + C_rb;

%% Righting moment
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

%% define D
 % bugbc! missing D_h is wrong p.5 right, Frh unknown
 D_h = [Frh(v_ah)*cos(alpha_ah);
     -Frh(v_ah)*sin(alpha_ah)*cos(phi);
     (-Frh(v_ah)*sin(alpha_ah)*cos(phi))*abs(z_h);
     Frh(v_ah)*sin(alpha_ah)*cos(phi)*abs(x_h);
 ];


% p.5 left
liftk=lift(rho_w,A_k,v_ak,alpha_ak);
dragk=drag(rho_w,A_k,v_ak,alpha_ak); % Use A_k or A_h??
 D_k = [-liftk*sin(alpha_ak) + dragk*cos(alpha_ak);
     -liftk*cos(alpha_ak) - dragk*sin(alpha_ak);
     (-liftk*cos(alpha_ak) - dragk*sin(alpha_ak))*abs(z_k);
     (liftk*cos(alpha_ak) + dragk*sin(alpha_ak))*abs(x_k);
 ];

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
% Sail Forces p.5 left
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

% p.4 left
v_dot = -inv(M)*C*vss-inv(M)*D-inv(M)*g+inv(M)*T;

dxdt = zeros(8,1);
dxdt(1:4) = n_dot;
dxdt(5:8) = v_dot;

end

