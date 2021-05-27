%Sampling and system constants
Ts = 0.5;
ny = 8; % number of output states 
nu = 4; % number of inputs
nx = 8; % number of system states
init_time = 0; % start time
final_time = 10; % end time
N = 1 + (final_time - init_time)/Ts; %number of samples
PureConstants; % updated constants file to only include constants and no derived constants or derived data structures
Parameters    = [m, I_xx, I_zz, I_xz, A_s, A_r, A_k, x_m, x_r, ...
                 y_r, z_r, x_s, y_s, z_s, x_h, y_h, z_h, x_k, y_k, ...
                 z_k, a_right, b_right, c_heel, d_yaw, k_drag, k_lift, ...
                 k_rh, X_u_dot, Y_v_dot, rho_w, rho_a, waterline_vol]; %first guess for estimated parameters
y = randn(N, ny); % replace with measured data
u = randn(N, nu); % replace with measured inputs and disturbances
z = iddata(y, u, Ts);
z.InputName = {'Sail Angle', 'Rudder Angle', 'Wind Speed', 'Wind Angle'};
z.InputUnit =  {'rad', 'rad', 'm/s', 'rad'};
z.OutputName = {'X', 'Y', 'phi', 'psi', 'surge', 'sway', 'roll', 'yaw' };
z.OutputUnit = {'m', 'm', 'rad', 'rad', 'm/s', 'm/s', 'rad/s', 'rad/s' };
z.Tstart = 0;
z.TimeUnit = 's';

FileName      = 'fullBoat_CTS2'; % ODE system       
Order         = [ny nu nx];      % system size      
         
InitialStates = [1;1;1;1;1;1;1;1];  % initial state of the system          
Ts            = 0;  % sanpling time is now 0 since we have a continuous time model

%idnlgrey creates the nonlinear greybox model
nlgr = idnlgrey('fullBoat_CTS2', Order, Parameters, InitialStates, Ts); 

%setting the name and unit of the inputs and outputs
set(nlgr, 'InputName', {'Sail Angle', 'Rudder Angle', 'Wind Speed', 'Wind Angle'}, 'InputUnit', {'rad', 'rad', 'm/s', 'rad'}',               ...
          'OutputName', {'X', 'Y', 'phi', 'psi', 'surge', 'sway', 'roll', 'yaw' }, ...
          'OutputUnit', {'m', 'm', 'rad', 'rad', 'm/s', 'm/s', 'rad/s', 'rad/s' },                         ...
          'TimeUnit', 's');
nlgr = setinit(nlgr, 'Name', {'X', 'Y', 'phi', 'psi', 'surge', 'sway', 'roll', 'yaw' });
nlgr = setinit(nlgr, 'Unit', {'m', 'm', 'rad', 'rad', 'm/s', 'm/s', 'rad/s', 'rad/s' });
nlgr.InputGroup.controls = [1 2];
nlgr.InputGroup.noise = [3 4];

%bool_est is set to false when a parameter is not to be estimated and is to
%be kept constant at the initial value ex. mass

            
 VALUES_BOOL = cell(32, 1);
 VALUES_BOOL(:) = {true};
 VALUES_BOOL(1) = {false};

%fixing the parameters as dictated by bool_est
nlgr = setpar(nlgr, 'Fixed', VALUES_BOOL);

%initial states could be fixed or not
nlgr = setinit(nlgr, 'Fixed', {false false false false false false false false}); % Estimate the initial states.

%weighting matrix to customise which output states should be traccked more
%accurately
W =  diag([1,1,1,1,1,1,1,1]);

opt = nlgreyestOptions('Display', 'on','OutputWeight',W);
% opt.SearchMethod = 'lsqnonlin';
% opt.SearchOptions.Advanced.UseParallel = true;
nlgr.SimulationOptions.Solver = 'ode23s';
size(nlgr)
nlgr = nlgreyest(z, nlgr, opt );

nlgr.Report
fprintf('\n\nThe search termination condition:\n')
nlgr.Report.Termination

compare(z, nlgr)

EstimateValues = cell(1,32);
for i = 1:32
    EstimatedValues(i) = nlgr.parameter(i).Value;
end

save('EstimatedValues.mat', 'EstimatedValues');

