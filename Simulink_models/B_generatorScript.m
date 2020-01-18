addpath("..\controller_design")
syms t x1 x2 x3 x4 x5 x6 x7 x8 u1 u2 w1 w2
symDE = nonLinearStateSpace([x1, x2, x3, x4, x5, x6, x7, x8], [u1, u2,w1 w2]);
B = jacobian(symDE, [u1,u2]);
matlabFunction(B,'File','B_gen','Optimize',false);