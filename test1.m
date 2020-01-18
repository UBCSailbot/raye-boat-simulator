function [Ad,Bd,Cd,Dd,U,Y,X,DX] = test1(u, x)
Ad = eye(8);
Bd = ones(8, 4);
Cc = eye(8);
Dc = zeros(8,4);
% Generate discrete-time model

Cd = Cc;
Dd = Dc;

% Nominal conditions for discrete-time plant
X = x;
U = u;

Y = Cd*x + Dd*u;
DX = Ad*x+Bd*u-x;