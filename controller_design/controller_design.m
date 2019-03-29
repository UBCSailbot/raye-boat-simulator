clear
clc
% Inverted Pendulum Parameters

M = 1;      % mass of the cart [kg]
m = 0.1;    % mass of the pendulum [kg]
l = 1;    % length of the pendulum [m]
g = 9.8;   % gravity acceleration [N/kg]

%% Linearized model x=[y,ydot,theta,thetadot]'
A = state_eq.A;
B = state_eq.B;
C = [1 0 0 0 0 0 0 0
    0 1 0 0 0 0 0 0];
D = [0 0; 0 0];

%% Contoller and observer design
%% Q1
polevec =[linspace(1,5,4)*i,linspace(-1,-5,4)*i]-5;
K = place(A,B,polevec);
% plot(tout,yout);
%return % Comment out this line if you solve Q2
%% Q2
Aaug = [A zeros(size(A,1),1);
    -C(1,:) 0];
Baug = [B; 0];
%polevec = [-2+2i -2-2i -2+i -2-i -2 ]; % [INPUT 1-BY-5 VECTOR HERE];
polevec = [-2+2i -2-2i -2+i -2-i -2 ]*1.2; % [INPUT 1-BY-5 VECTOR HERE];
Kaug = place(Aaug,Baug,polevec);
K = Kaug(1:4); Ka = Kaug(5);