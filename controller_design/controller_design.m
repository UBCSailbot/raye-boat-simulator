clear
clc

[parentdir,~,~]=fileparts(pwd);
load(fullfile(parentdir,"equations.mat"));

[nx,~]=size(ss_eq);
ny=nx;
nu=2;

nlobj = nlmpc(nx,ny,'MV',[1,2],'MD',[3,4])

Ts = 0.1;
nlobj.Ts = Ts;

nlobj.PredictionHorizon = 10;
nlobj.ControlHorizon = 5;

nlobj.Model.StateFcn = "boat_state_function";
nlobj.Model.IsContinuousTime = true;


nlobj.Model.NumberOfParameters = 0;

nlobj.Model.OutputFcn = @(x,u) [x(:)];

nlobj.Weights.OutputVariables = ones(1,8)*3;
nlobj.Weights.ManipulatedVariablesRate = ones(1,2)*0.1;

x0=ones(8,1)
u0=[2,2]
validateFcns(nlobj, x0, u0, u0);
