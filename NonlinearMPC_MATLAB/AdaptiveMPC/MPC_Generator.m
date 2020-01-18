%% create MPC controller object with sample time
mpc2 = mpc(plant_C, 1);
%% specify prediction horizon
mpc2.PredictionHorizon = 10;
%% specify control horizon
mpc2.ControlHorizon = 2;
%% specify nominal values for inputs and outputs
mpc2.Model.Nominal.U = [0;0;0];
mpc2.Model.Nominal.Y = [6.84314609277491;-4.86067542619167;-0.0040903484771527;-1.96661094271451;0.121852789883292;0.129749665720824;0.00509046345493348;-0.0115772583040515];
%% specify overall adjustment factor applied to weights
beta = 0.64404;
%% specify weights
mpc2.Weights.MV = [0 0]*beta;
mpc2.Weights.MVRate = [0.1 0.1]/beta;
mpc2.Weights.OV = [0 0 0 1 0 0 0 0]*beta;
mpc2.Weights.ECR = 100000;
%% specify simulation options
options = mpcsimopt();
options.RefLookAhead = 'off';
options.MDLookAhead = 'off';
options.Constraints = 'on';
options.OpenLoop = 'off';
%% run simulation
sim(mpc2, 11, mpc2_RefSignal, mpc2_MDSignal, options);
