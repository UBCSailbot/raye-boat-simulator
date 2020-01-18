init = [0;0;0;0;0];
t_final = 10;
t_start = 0;

Ts = 0.5;
N = (t_final-t_start)/Ts + 1;
x = zeros(5, N+1);
for i = 2:N+1
    x(:, i) = simpleBoat_DT(x(:, i-1)', [pi/4, pi/4]);
end
plot(x)

    