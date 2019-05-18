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


nlobj.Model.NumberOfParameters = 1;

nlobj.Model.OutputFcn = @(x,u,Ts) [x(:)];

nlobj.Weights.OutputVariables = ones(1,8)*3;
nlobj.Weights.ManipulatedVariablesRate = ones(1,2)*0.1;

validateFcns(nlobj,ones(8,1),[2,2],[2,2],[],{Ts})