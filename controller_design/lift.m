function force = lift(rho,A,v,alpha)
% Calculate drage force
% NOTE: bugbc C_L(alpha) was guessed to be k1*sin(2*alpha)
  load('..\boat_constants.mat')
  force = 0.5*rho*A*(v^2)*k_lift*sin(2*alpha);
end