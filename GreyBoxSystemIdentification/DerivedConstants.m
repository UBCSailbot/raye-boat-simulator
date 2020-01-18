%Derived constants
xyz_s = [x_s;y_s;z_s];  % CoE sail
xyz_r = [x_r;y_r;z_r];  % CoE rudder
xyz_h = [x_h;y_h;z_h];  % CoE hull
x_sm = norm(xyz_s);
added_mass = waterline_vol * rho_w /2; % Added mass phenomena <-not sure where this is from

M_rb = [m 0   0     0;
        0 m   0     0;
        0 0  I_xx -I_xz;
        0 0 -I_xz I_zz]; %bugbc check if Ixz = 0
M_add = diag(added_mass*ones(1,4)); % bugbc double check that this approximation is right: p.6 left M_A = -diag{X_u_dot, Y_v_dot, K_p_dot, N_r_dot}, on p.3 it says M_A is stricly positive 
M = M_rb + M_add; 