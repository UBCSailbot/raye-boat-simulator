function force = drag(rho,A,v,alpha)
% Calculate drage force
% NOTE: bugbc C_D(alpha) was guessed to be k1*(1-cos(2*alpha))
  k1 = 1;
  force = 0.5*rho*A*(v^2)*k1*(1-cos(2*alpha));
end

