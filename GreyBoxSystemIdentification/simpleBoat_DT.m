function xk1 = simpleBoat_DT(xk, uk)
Ts = 0.5; %sampling time
M = 10;
t = 0;
delta = Ts/M;
xk1 = xk;
for ct=1:M
    xk1 = xk1 + delta*simpleBoat_CTS(t, xk1,uk, 0.03);
end