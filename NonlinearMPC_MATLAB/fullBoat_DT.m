function xk1 = fullBoat_DT(xk, uk)
Ts = 0.1; %sampling time
M = 10;
delta = Ts/M;
xk1 = xk;
for ct=1:M
    xk1 = xk1 + delta*fullBoat_CTS(xk1,uk);
end