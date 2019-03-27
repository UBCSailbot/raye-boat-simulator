load('inputData.mat');
load('equations.mat');

n_in = [x_in,y_in,phi_in,psi_in];
v_in = transpose(eval(subs(J,[psi,phi],[psi_in,phi_in])\[dx_in;dy_in;roll_in;yaw_in]));
ang_in = [sangle_in,rangle_in];

linearized_A = jacobian(ss_eq,ss_vars);
linearized_B_ctrl = jacobian(ss_eq,ss_in_controllable);
linearized_B_disturb = jacobian(ss_eq,ss_in_disturbance);
%begin subs 
%state variables
x=0; y=0; phi=0; psi=0; surge=1; sway=0; roll=0; yaw=0;
%ss_in_contrrolable
sangle=0; rangle=0;
%ss_in_disturbance;
v_tw=0; alpha_tw=0;

linearized_A = subs(linearized_A);
linearized_B_ctrl = subs(linearized_B_ctrl);
linearized_B_disturb = subs(linearized_B_disturb);

C = [1 0 0 0 0 0 0 0;
    0 1 0 0 0 0 0 0;
    0 0 1 0 0 0 0 0;
    0 0 0 1 0 0 0 0];

D = zeros(4,2);

state_eq = ss(eval(real(linearized_A)), eval(real(linearized_B_ctrl)), C, D);
% Set the Inputs and Output names of the model with units
state_eq.InputName = {'sail','rudder'};
state_eq.InputUnit={'rad','rad'};
state_eq.OutputName = {'dx','dy','dphi','dpsi'};
state_eq.OutputUnit={'m/s','m/s','rad/s','rad/s'};

%% Save the Model
save state_eq.mat state_eq

%% check stability, controllability, observability, stabilizability of linear model
eig(state_eq.A); %not stable
rank(ctrb(state_eq.A,state_eq.B))%fully controllable
rank(obsv(state_eq.A,state_eq.C));%fully observable

%% create non linear model block for simulink
get_non_linear_states=matlabFunction(ss_eq);


