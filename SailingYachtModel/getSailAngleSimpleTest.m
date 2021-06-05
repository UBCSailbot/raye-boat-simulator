fplot(@(z) getSailAngleSimple(z), [-pi, pi]);
assert(getSailAngleSimple(pi/4) < pi);
assert(getSailAngleSimple(pi/2) <= pi/2 && getSailAngleSimple(pi/2) > pi/5);
assert(getSailAngleSimple(0) > pi/4);