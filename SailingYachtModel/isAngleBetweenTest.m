assert(isAngleBetween(1,3,2) == true);
assert(isAngleBetween(3,1,2) == true);

assert(isAngleBetween(-1,1,0) == true);

assert(isAngleBetween(-1,1,0.1) == true);

assert(isAngleBetween(-1,1,-0.1) == true);

assert(isAngleBetween(0,pi,pi/6) == true);
assert(isAngleBetween(0,pi,-pi/6) == true);

assert(isAngleBetween(1,2,3) == false);

