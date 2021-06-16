fplot(@(z) getSailAngleRadSimple(z), [-pi, pi]);
assert(getSailAngleRadSimple(pi/4) < pi);
assert(getSailAngleRadSimple(pi/2) <= pi/2 && getSailAngleRadSimple(pi/2) > pi/5);
assert(getSailAngleRadSimple(0) > pi/4);