%% Record Time Evolution Data

close all;
clear all;

filename = 'dummytest';

run('nonlinear_sailboat_motion.m');

dt = 0.1;
data = [x_in;y_in;phi_in;psi_in];
save(filename,'data');
time_step = 1;

for i = 0:dt:10
    time_step = time_step + 1;
    run('ss_linearization.m');
    load state_eq.mat
    open_system('boat_sim_ss');
    sim('boat_sim_ss');
    close_system('boat_sim_ss');
    run('updateData.m');
    m = matfile(filename,'Writable',true);
    m.data(:,time_step) = [x_in;y_in;phi_in;psi_in];
    data = m.data;
    save(filename,'data');
end