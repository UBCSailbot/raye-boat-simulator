nx = 8; % number of states on the simplified boat system
ny = 8; % only output is yaw
nu = 2; % two inputs: rudder and sail angle

nlobj = nlmpc(nx, ny, 'MV', [1, 2], 'MD', [3,4]); % create the non linear MPC

Ts = 0.5; 
nlobj.Ts = Ts; % set the sampling time of the nonlinear MPC

nlobj.PredictionHorizon = 2; % Prediction horizon is the number of time steps a control action is analysed over
nlobj.ControlHorizon = [1 1]; % Control Horizon is [1, Prediction Horizon]

nlobj.Model.StateFcn = "fullBoat_DT"; % give the ODE system of our boat to the Nonlinear MPC
nlobj.Model.IsContinuousTime = false;

nlobj.Model.OutputFcn = "fullBoat_DT"; %output the yaw 

nlobj.Weights.OutputVariables = [1 1 1 100000 1 1 1 1]; % weight of the output (yaw) in the cost function, 3 seems to be the standard value for MPC
nlobj.Weights.ManipulatedVariablesRate = [10, 10]; %rate of change of output

nlobj.MV(1).Min = -pi; %minimum sail angle
nlobj.MV(1).Max = pi;  %maximum sail angle

nlobj.MV(2).Min = -pi/3; %minimum rudder angle
nlobj.MV(2).Max = pi/3;  %maximum rudder angle

x0 = [0, 0, 0, 0, 0, 0, 0, 0];
u0 = [0, 0, 0, 0];
validateFcns(nlobj, x0, u0(1:2), u0(3:4));

EKF = extendedKalmanFilter(@fullBoat_DT, @boatMeasurementFcn);

x = [0;0;0;0;0;0;0;0]; %initial state
y = x;
EKF.State = x;
mv = [0, 0];
y_ref = [1,1,1,-0.3,1,1,1,1]; % reference yaw angle

% x0 = ones(1,8);
% u0 = ones(1,4);
% validateFcns(nlobj, x0, u0(1:2), u0(3:4));

nloptions = nlmpcmoveopt;

Duration = 10;
hbar = waitbar(0,'Simulation Progress');
xHistory = x;
mvHistory = [0;0];
for ct = 1:(Duration/Ts)
     xk = correct(EKF, y);
     [mv,nloptions,info] = nlmpcmove(nlobj,x,mv,y_ref,[5, -pi/6],nloptions);
     predict(EKF, [[mv;5;-pi/6]; Ts]);
      x = fullBoat_DT(x,[mv;5;-pi/6]);
      y = x + randn*0.00001;
      waitbar(ct*Ts/Duration,hbar);
      xHistory = [xHistory x];
      mvHistory = [mvHistory mv];
end
close(hbar);


plot(0:Ts:Duration,mvHistory(1,:))
xlabel('time')
ylabel('sail angle')
title('sail angle of the boat')
plot(0:Ts:Duration,xHistory(4,:))

%plot(0:Ts:Duration,mvHistory(1,:))
xlabel('time')
ylabel('yaw')
title('heading of the boat')