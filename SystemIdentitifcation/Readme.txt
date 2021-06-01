Estimate values by running System Identification program

use y to input measured state data and u for system inputs

All values set to False in VALUES_BOOL will be estimated, all set to true will be fixed to their initial values

The 31 array indices in VALUES_BOOL correspond respectively to:

 mass, I_xx, I_zz, I_xz, A_s, A_r, A_k, x_m,
 x_r y_r z_r x_s y_s z_s x_h y_h, z_H, x_k,
 y_k, z_k, a_right, b_right, c_heel, d_yaw,
 k_drag, k_lift, k_rh, X_u_dot, rho_w, rho_a, waterline_vol

Estimated parameter values will be output to EstimatedValues.mat file in same order of parameters, including both estimated and fixed values.