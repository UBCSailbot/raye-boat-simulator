%note runing this the first time after openiang
%MATLAB takes a while (~ 1.5 minutes for Bruce 

%define constants. Modify these as applicable
global par
par.m = 25900;                         % (kg),mass of the vehicle
par.Ixx = 133690;          
par.Izz = 24760;      
par.Ixz = 2180;                        % moment of inertia
par.a11 = 970;   
par.a22 = 17430; 
par.a44 = 106500;
par.a66 = 101650;
par.a24 = -13160;
par.a26 = -6190; 
par.a46 = 4730;                        % (kg),added mass coef.

% see here for sensor conventions:
%https://ubcsailbot.atlassian.net/wiki/spaces/ADA2/pages/1462599682/Software+Sensor+Units+and+Convention

par.vt = 5;                           % the norm of wind velocity 
par.alpha_w = pi/4;                      % the direction of wind 

par.rho_a = 1.2;                       % (kg/m^3), air density
par.As = 170;                          % (m^2), sail area
par.h0 = 0.0005;                       % (m), roughness height
par.h1 = 11.58;                        % (m), reference height
par.z_s = -11.58;                      % (m), (x,y,z) is the CoE
par.xs = 0;
par.ys = 0;
par.zs = -11.58;                       % (m), (x,y,z) is the CoE
par.Xce = 0.6;                         % (m), distance along the mast to the CoE
par.Xm = 0.3;                          % (m), x-coordinate of the mast 

par.rho_w = 1025;                      % (kg/m^3), water density
par.Ar = 1.17;                         % (m^2), rudder area
par.d_r = 1.9;                         % rudder draft
par.zeta_r = 0.8;                      % rudder efficiency
par.x_r = -8.2;
par.z_r = -0.78;                       % (m), (x,y,z) is the CoE
par.xr = -8.2;
par.yr = 0;
par.zr = -0.78;                        % (m), (x,y,z) is the CoE

par.Ak = 8.7;                          % (m^2), keel area
par.d_k = 2.49;                        % keel draft
par.zeta_k = 0.7;                      % keel efficiency
par.x_k = 0;
par.z_k = -0.58;                       % (m), (x,y,z) is the CoE
par.xk = 0;
par.yk = 0;
par.zk = -0.58;                        % (m), (x,y,z) is the CoE

par.x_h = 0;
par.z_h = -1.18;                       % (m), (x,y,z) is the CoE
par.xh = 0;
par.yh = 0;
par.zh = -1.18;                        % (m), (x,y,z) is the CoE

par.w_c = 60000;                       % (N), crew weight 20000
par.x_c = -8;                          % (m), crew position
par.y_bm = 3.6;                        % (m), yacht beam

par.a = -5.89;
par.b = 8160;
par.c = 120000;
par.d = 50000;

set_param('boat_simulator_ros_node', 'SimulationMode', 'normal')
load_system('boat_simulator_ros_node.slx');
sim boat_simulator_ros_node.slx;