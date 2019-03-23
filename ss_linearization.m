load('inputData.mat');
load('equations.mat');

n_in = [x_in,y_in,phi_in,psi_in];
v_in = transpose(eval(subs(J,[psi,phi],[psi_in,phi_in])\[dx_in;dy_in;roll_in;yaw_in]));
ang_in = [sangle_in,rangle_in];

linearized_A = jacobian(ss_eq,ss_vars);
linearized_B = jacobian(ss_eq,ss_in);

linearized_A = subs(linearized_A,[ss_vars, ss_in], [n_in, v_in, ang_in]);
linearized_B = subs(linearized_B,[ss_vars, ss_in], [n_in, v_in, ang_in]);

C = [1 0 0 0 0 0 0 0;
    0 1 0 0 0 0 0 0;
    0 0 1 0 0 0 0 0;
    0 0 0 1 0 0 0 0];

D = zeros(4,2);

state_eq = ss(eval(real(linearized_A)), eval(real(linearized_B)), C, D);
% Set the Inputs and Output names of the model with units
state_eq.InputName = {'sail','rudder'};
state_eq.InputUnit={'rad','rad'};
state_eq.OutputName = {'dx','dy','dphi','dpsi'};
state_eq.OutputUnit={'m/s','m/s','rad/s','rad/s'};

%% Save the Model
save state_eq.mat state_eq

%% check stability, controllability, observability, stabilizability
 
eig(state_eq.A); %not stable
rank(ctrb(state_eq.A,state_eq.B))%fully controllable
rank(obsv(state_eq.A,state_eq.C));%fully observable

