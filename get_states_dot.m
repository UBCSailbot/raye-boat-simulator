function output=get_states_dot(alpha_tw, phi, psi, rangle, roll, sangle, surge, sway, v_tw, yaw)
% alpha_tw=1; phi=1; psi=1; rangle=1; roll=1; sangle=1; surge=1; sway=1; v_tw=1; yaw=1;
    load equations.mat ss_eq
    
    output=eval(subs(ss_eq))


end