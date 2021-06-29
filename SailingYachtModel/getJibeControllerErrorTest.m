assert(getJibeControllerErrorRad(pi /2, - pi / 2, 0) == pi);
assert(getJibeControllerErrorRad(pi /2, - pi / 2, pi) == -pi);
assert(getJibeControllerErrorRad((pi /2) + 2 * pi, - pi / 2 + 2 * pi, 0) == pi);
assert(getJibeControllerErrorRad((pi /2) + 2 * pi, - pi / 2 + 2 * pi, pi) == -pi);
assert(getJibeControllerErrorRad(pi /100, - pi / 100, 0) == (99/100) * 2 * pi);
assert(getJibeControllerErrorRad(pi /100, - pi / 100, pi) > (-3/100) * pi && getJibeControllerErrorRad(pi /100, - pi / 100, pi) < 0);