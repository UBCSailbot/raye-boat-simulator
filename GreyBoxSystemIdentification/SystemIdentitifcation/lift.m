function force = lift(rho,A,v,alpha, k_lift)
% Calculate drage force
% NOTE: bugbc C_L(alpha) was guessed to be k1*sin(2*alpha)
%Constants_Oct_15_2019
  force = 0.5*rho*A*(v^2)*k_lift*sin(2*alpha);
end