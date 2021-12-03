%note runing this the first time you start simulink afte ropeniangMATLAB
%takes a hilwe (~ 1.5 minutes for Bruce)
set_param('boat_simulator_ros_node', 'SimulationMode', 'normal')
sim boat_simulator_ros_node.slx;