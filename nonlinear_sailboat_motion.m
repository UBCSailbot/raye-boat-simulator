% Non-Lenear model of Autonomous Sailing Vessel
% UBC Sailbot
% Nicolas Navarre
% Feb 25 2019
close all;
clear all;
%% State Equation Setup

%Define constants
m = 1000; %kg
I_xx = 100; %kg m^2
I_zz = 100; %kg m^2
A_s = 10; %m^2
A_r = 1; %m^2
rCoE_x = 0.1; %m
rCoE_z = 0.3; %m
sCoE_x = 1; %m
sCoE_z = 3; %m
rho_w = 997; %kg/m^3
rho_a = 1.225; %kg/m^3
k1 = 1; %lift/drag const (This can be tuned)
waterline_vol = 2; %m^3  This might have to be a function of roll
added_mass = waterline_vol * rho_w /2; %

%State space variables
syms x y yaw roll surge sway droll dyaw dx dy sangle rangle

%Variable Constants
syms w_abs w_rel v_abs v_rel lift drag hdamp A rho alpha v du dv ddroll ddyaw


run('generateData.m');
load('inputData.mat');

if yaw_in > 0 
    w_abs = -(w_rel_in - yaw_in);
else
    w_abs = (w_rel_in - yaw_in);
end


%% Necessary motion functions

surge(dx,dy,yaw) = dx*sin(yaw) + dy*sin(yaw);
sway(dx,dy,yaw) = dx*cos(yaw) - dy*sin(yaw);
lift(rho,A,v,alpha) = 0.5*rho*A*(v.^2)*k1*sin(2*alpha);
drag(rho,A,v,alpha) = 0.5*rho*A*(v.^2)*k1*(1-cos(2*alpha));


%% Collection of Sensor Data
% Input valiables may be randomly generated as they would be measured on
% the boat
