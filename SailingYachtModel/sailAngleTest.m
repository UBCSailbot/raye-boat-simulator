
fplot(@(x) sailAngle(x), [-pi, pi]);

assert(sailAngle(pi/4) < pi);
assert(sailAngle(pi/2) <= pi/2 && sailAngle(pi/2) > pi/5);
assert(sailAngle(0) > pi/4);
