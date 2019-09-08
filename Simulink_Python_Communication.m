% Clear and reset
clc;
clear;
close all;

% Constants
sim_time_step = 0.0001;

% Open Simulink Model
Constants_Aug_2019
open_system('Simulink_models/sailbot_library.slx')
open_system('Simulink_models/Assemble_Blocks_Aug_24.slx')

% start the simulation and pause the simulation, waiting for signal from python
set_param('Assemble_Blocks_Aug_24','SimulationCommand','start','SimulationCommand','pause');

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
    x_block = get_param('Assemble_Blocks_Aug_24/x_dot_to_x', 'RuntimeObject');
    y_block = get_param('Assemble_Blocks_Aug_24/y_dot_to_y', 'RuntimeObject');
    phi_block = get_param('Assemble_Blocks_Aug_24/phi_dot_to_phi', 'RuntimeObject');
    psi_block = get_param('Assemble_Blocks_Aug_24/psi_dot_to_psi', 'RuntimeObject');
    
    if or(or(isempty(x_block), isempty(y_block)), or(isempty(phi_block), isempty(psi_block)))
        continue
    else
        % Get useful variables
        x = x_block.OutputPort(1).Data;
        y = y_block.OutputPort(1).Data;
        phi = phi_block.OutputPort(1).Data;
        psi = psi_block.OutputPort(1).Data;
        
        x_dot = x_block.InputPort(1).Data;
        y_dot = y_block.InputPort(1).Data;
        phi_dot = phi_block.InputPort(1).Data;
        psi_dot = psi_block.InputPort(1).Data;
        
        u = [x, y, phi, psi, x_dot, y_dot, phi_dot, psi_dot]
    end
    
    % Send information to Python
    fwrite(s, jsonencode(u));
    
end
fclose(s);

