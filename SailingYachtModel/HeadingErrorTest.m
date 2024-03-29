
testRange(7 * pi / 8, -7*pi/8, -pi / 4, 0.01);

testRange(-7 * pi / 8, 7*pi/8, pi / 4, 0.01);

testRange(pi / 2, 0, pi / 2, 0.01);

testRange(0, pi/2, -pi / 2, 0.01);

testRange(pi/4, -pi/4, pi/2, 0.01);

testRange(pi/3, -3 * pi/4, -11 * pi / 12, 0.01);

testRange(4 * pi, -pi/4 + 2*pi, pi/4, 0.001);

testRange(3 * pi, 1.1 * pi, -0.1 * pi, 0.001);

function testRange(setPoint, measure, target, range)
    error = getHeadingError(setPoint, measure);
    assert(error > target - range && error < target + range);
end

