assert(getJibeControllerDirection(pi /2, - pi / 2, 0) == -1);
assert(getJibeControllerErrorRad(pi /2, - pi / 2, -1) == -pi);

assert(getJibeControllerDirection(pi /2, - pi / 2, pi) == 1);
assert(getJibeControllerErrorRad(pi /2, - pi / 2, 1) == pi);

assert(getJibeControllerDirection((pi /2) + 2 * pi, - pi / 2 + 2 * pi, 0) == -1);
assert(getJibeControllerErrorRad((pi /2) + 2 * pi, - pi / 2 + 2 * pi, -1) == -pi);

assert(getJibeControllerDirection((pi /2) + 2 * pi, - pi / 2 + 2 * pi, pi) == 1);
assert(getJibeControllerErrorRad((pi /2) + 2 * pi, - pi / 2 + 2 * pi, 1) == pi);

assert(getJibeControllerDirection(pi /100, - pi / 100, pi) == 1);
assert(getJibeControllerErrorRad(pi /100, - pi / 100, 1) == (99/100) * 2 * pi);

