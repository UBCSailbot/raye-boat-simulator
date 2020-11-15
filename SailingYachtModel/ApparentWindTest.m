assert(apparentWind(0.5, 0, 2, 0, 0) == 0);
assert(apparentWind(0,0,1,pi/2,0) == pi/2);
assert(apparentWind(1,0,10,pi/2,0) > pi/2);