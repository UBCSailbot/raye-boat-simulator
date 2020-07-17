%% Simulink_Python_Communcation.m
% % Simple script to get Simulink Model => MATLAB Script => Python Script => Gazebo
% % Full instructions:
% % 
% % 1. Install MATLAB, Simulink and Python
% % 2. Install ROS, Gazebo and VRX (https://bitbucket.org/osrf/vrx/wiki/tutorials/SystemSetupInstall)
% % 3. Run roslaunch vrx_gazebo sandisland.launch (may need to run source ~/vrx_ws/devel/setup.bash first). Pause the simulation
% % 4. Run Simulink_Python_Communcation.m
% % 5. Run python simulink_to_gazebo_pose.py from cmd line
% % 6. View Gazebo to see the boat moving

% Clear and reset
clc;
clear;
close all;

% Constants
sim_time_step = 0.0001;

% Open Simulink Model
Constants_Oct_15_2019
open_system('Simulink_models/sailbot_library.slx')
open_system('Simulink_models/Assemble_Blocks_Oct_15.slx')

% start the simulation and pause the simulation, waiting for signal from python
set_param('Assemble_Blocks_Oct_15','SimulationCommand','start','SimulationCommand','pause');

% open a server, it will block until a client connect to it
s = tcpip('127.0.0.1', 54320,  'NetworkRole', 'server');
fopen(s);

% Wait for start command from Python
while(1)
    nBytes = get(s,'BytesAvailable');
    if nBytes>0
        break;
    end
end

% main loop
while(1)     
    % run the simulink model for a step
    set_param(gcs, 'SimulationCommand', 'step');  
    
    % pause the simulink model and send some data to python
    pause(sim_time_step);
    
    % Get block objects
    x_block = get_param('Assemble_Blocks_Oct_15/x_dot_to_x', 'RuntimeObject');
    y_block = get_param('Assemble_Blocks_Oct_15/y_dot_to_y', 'RuntimeObject');
    phi_block = get_param('Assemble_Blocks_Oct_15/phi_dot_to_phi', 'RuntimeObject');
    psi_block = get_param('Assemble_Blocks_Oct_15/psi_dot_to_psi', 'RuntimeObject');
    sangle_block = get_param('Assemble_Blocks_Oct_15/sangle_block', 'RuntimeObject');
    rangle_block = get_param('Assemble_Blocks_Oct_15/rangle_block', 'RuntimeObject');
    v_tw_block = get_param('Assemble_Blocks_Oct_15/v_tw_block', 'RuntimeObject');
    alpha_tw_block = get_param('Assemble_Blocks_Oct_15/alpha_tw_block', 'RuntimeObject');
    
    % Incomplete: handle case where there is bad data, which crashes
    % simulation
    if or(or(isempty(x_block), isempty(y_block)), or(isempty(phi_block), isempty(psi_block)))
        continue
    else
        % Get useful variables
        x = x_block.OutputPort(1).Data;
        y = y_block.OutputPort(1).Data;
        phi = phi_block.OutputPort(1).Data;
        psi = psi_block.OutputPort(1).Data;
        
        sangle = sangle_block.OutputPort(1).Data;
        rangle = rangle_block.OutputPort(1).Data;
        v_tw = v_tw_block.OutputPort(1).Data;
        alpha_tw = alpha_tw_block.OutputPort(1).Data;
        
        u = [x, y, phi, psi, sangle, rangle, v_tw, alpha_tw]
    end
    
    % Send information to Python
    fwrite(s, jsonencode(u));
    
end
fclose(s);

