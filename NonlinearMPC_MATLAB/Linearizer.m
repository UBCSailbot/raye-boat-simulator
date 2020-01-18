function [Ad,Bd,Cd,Dd,U,Y,X,DX] = Linearizer(u, x)
for i = 1:8
    if(x(i) == 0)
        x(i) = 0.01;
    end
end
u1 = u(1);
u2 = u(2);
u3 = u(3);
u4 = u(4);
x1 = x(1);
x2 = x(2);
x3 = x(3);
x4 = x(4);
x5 = x(5);
x6 = x(6);
x7 = x(7);
x8 = x(8);

Ac = real(A_func(u1,u2,u3,u4,x3,x4,x5,x6,x7,x8));
Bc = real(B_func(u1,u2,u3,u4,x3,x4,x5,x6,x7,x8));

Ts = 1;
Cc = eye(8);
Dc = zeros(8,4);
% Generate discrete-time model
nx = size(Ac,1);
nu = size(Bc,2);
M = expm([[Ac Bc]*Ts; zeros(nu,nx+nu)]);
Ad = M(1:nx,1:nx);
Bd = M(1:nx,nx+1:nx+nu);
Cd = Cc;
Dd = Dc;

% Nominal conditions for discrete-time plant
X = x;
U = u;
size(Cd)
size(Dd)
size(u)
Y = Cd*x + Dd*u;
DX = Ad*x+Bd*u-x;



