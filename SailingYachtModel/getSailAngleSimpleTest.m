fplot(@(z) getSailAngleRadSimple(z), [-2 * pi, 2 * pi]);
assert(getSailAngleRadSimple(pi/4) < pi);
assert(getSailAngleRadSimple(pi/2) <= pi/2 && getSailAngleRadSimple(pi/2) > pi/5);
assert(getSailAngleRadSimple(0) > pi/4);
assert(getSailAngleRadSimple(10 * pi) >= 0 && getSailAngleRadSimple(10 * pi) <= pi /2);