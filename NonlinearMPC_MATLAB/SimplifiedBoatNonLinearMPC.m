nx = 5; % number of states on the simplified boat system
ny = 1; % only output is yaw
nu = 2; % two inputs: rudder and sail angle

nlobj = nlmpc(nx, ny, nu); % create the non linear MPC

Ts = 0.1; 
nlobj.Ts = Ts; % set the sampling time of the nonlinear MPC

nlobj.PredictionHorizon = 10; % Prediction horizon is the number of time steps a control action is analysed over
nlobj.ControlHorizon = 5; % Control Horizon is [1, Prediction Horizon]

nlobj.Model.StateFcn = "simpleBoat_DT"; % give the ODE system of our boat to the Nonlinear MPC
nlobj.Model.IsContinuousTime = false;

nlobj.Model.OutputFcn = @(x,u) [x(3)]; %output the yaw 

nlobj.Weights.OutputVariables = [3]; % weight of the output (yaw) in the cost function, 3 seems to be the standard value for MPC
nlobj.Weights.ManipulatedVariablesRate = [0.1, 0.1]; %rate of change of output

nlobj.MV(1).Min = -pi/3; %minimum rudder angle
nlobj.MV(1).Max = pi/3;  %maximum rudder angle

nlobj.MV(2).Min = -pi; %minimum sail angle
nlobj.MV(2).Max = pi;  %maximum sail angle

x0 = [0.1, 0.1, 0.5, 1, 0.01];
u0 = [0, 0];
validateFcns(nlobj, x0, u0, []);

EKF = extendedKalmanFilter(@simpleBoat_DT, @boatMeasurementFcn);

x = [0;0;0;0;0]; %initial state
y = [x(3)];
EKF.State = x;
mv = [0, 0];
y_ref = pi/3; % reference yaw angle

nloptions = nlmpcmoveopt;

Duration = 50;
hbar = waitbar(0,'Simulation Progress');
xHistory = x;
for ct = 1:(Duration/Ts)
     xk = correct(EKF, y);
     [mv,nloptions,info] = nlmpcmove(nlobj,x,mv,y_ref,[],nloptions);
     predict(EKF, [mv; Ts]);
      x = simpleBoat_DT(x,mv);
      y = x(3) + randn*0.01;
      waitbar(ct*Ts/Duration,hbar);
      xHistory = [xHistory x];
end
close(hbar);

plot(0:Ts:Duration,xHistory(3,:))
xlabel('time')
ylabel('yaw')
title('heading of the boat')

