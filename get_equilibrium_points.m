
eqns=[ss_eq(1)==0
    ss_eq(2)==0
    ss_eq(3)==0
    ss_eq(4)==0
    ss_eq(5)==0
    ss_eq(6)==0
    ss_eq(7)==0
    ss_eq(8)==0 
    alpha_tw==0
    v_tw==1
    
];

S = solve(eqns,'IgnoreAnalyticConstraints', true)
S=vpasolve(eqns)

  S.alpha_tw
         S.phi
         S.psi
      S.rangle
        S.roll
      S.sangle
       S.surge
        S.sway
        S.v_tw
         S.yaw

