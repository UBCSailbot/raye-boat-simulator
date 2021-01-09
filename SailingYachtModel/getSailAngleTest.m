%fplot(@(x) getSailAngle(x), [-pi, pi]);

assert(getSailAngle(pi/4) < pi);
assert(getSailAngle(pi/2) <= pi/2 && getSailAngle(pi/2) > pi/5);
assert(getSailAngle(0) > pi/4);
