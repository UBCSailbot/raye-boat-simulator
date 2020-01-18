syms t x1 x2 x3 x4 x5 x6 x7 x8 u1 u2 u3 u4
symDE = fullBoat_CTS(t, [x1, x2, x3, x4, x5, x6, x7, x8], [u1, u2, u3, u4]);
A = jacobian(symDE, [x1, x2, x3, x4, x5, x6, x7, x8]);
matlabFunction(A,'File','A_func','Optimize',false);