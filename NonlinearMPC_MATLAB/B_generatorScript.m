syms t x1 x2 x3 x4 x5 x6 x7 x8 u1 u2 u3 u4
symDE = fullBoat_CTS(t, [x1, x2, x3, x4, x5, x6, x7, x8], [u1, u2, u3, u4]);
B = jacobian(symDE, [u1, u2, u3, u4]);
matlabFunction(B,'File','B_func','Optimize',false);