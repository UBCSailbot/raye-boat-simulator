function xk1 = fullBoat_DT(xk, uk)
Ts = 0.5; %sampling time
M = 1;
delta = Ts/M;
xk1 = xk;
for ct=1:M
    xk1 = xk1 + delta*nonLinearStateSpace(xk1,uk);
end