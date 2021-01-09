assert(getApparentWindAngle(0.5, 0, 2, 0, 0) == 0);
assert(getApparentWindAngle(0,0,1,pi/2,0) == pi/2);
assert(getApparentWindAngle(1,0,10,pi/2,0) > pi/2);