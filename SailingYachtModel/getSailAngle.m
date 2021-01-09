
function [maxAngle] = getSailAngle(apparentWindAngle)
    boatAngle = pi - abs(apparentWindAngle);
    angleSet = 0:pi/100:pi;
    [Cls, Cds] = sailcoef(angleSet);
    mag = (Cls .*sin(boatAngle)) - (Cds .*cos(boatAngle));
    maxAngle = max(mag(:));
end


%------------------------------------------------------------------------------------------------------------------------------
function [Cls,Cds] = sailcoef(attack)
% generate a lookup table for the lift/drag coefficients for the sail
% and compute Cls/Cds from the lookup table using interpolation.

% lookup table
xdata = linspace(-pi,pi,73); % every 5 degrees
xdata = xdata/pi*180;

% lift curve
yldata = [fliplr([0 0.15 0.32 ...
    0.48 0.7 0.94 1.15 1.3 1.28 1.15 1.1 1.05 1 0.9 0.82 0.72 0.68 0.56 0.48 0.32 0.21 0.08 ...
    -0.06 -0.18 -0.3 -0.4 -0.53 -0.64 -0.72 -0.84 -0.95 -1.04 -1.1 -1.14 -1.08 -0.76 0])*(-1) ...
    [0.15 0.32 ...
    0.48 0.7 0.94 1.15 1.3 1.28 1.15 1.1 1.05 1 0.9 0.82 0.72 0.68 0.56 0.48 0.32 0.21 0.08 ...
    -0.06 -0.18 -0.3 -0.4 -0.53 -0.64 -0.72 -0.84 -0.95 -1.04 -1.1 -1.14 -1.08 -0.76 0]];

% drag curve
yddata = [fliplr([0.1 0.12 0.14 ...
    0.16 0.19 0.26 0.35 0.46 0.54 0.62 0.7 0.78 0.9 0.98 1.04 1.08 1.16 1.2 1.24 1.26 ...
    1.28 1.34 1.36 1.37 1.33 1.31 1.28 1.26 1.25 1.2 1.1 1.04 0.88 0.8 0.64 0.38 0.1])...
    [0.12 0.14 ...
    0.16 0.19 0.26 0.35 0.46 0.54 0.62 0.7 0.78 0.9 0.98 1.04 1.08 1.16 1.2 1.24 1.26 ...
    1.28 1.34 1.36 1.37 1.33 1.31 1.28 1.26 1.25 1.2 1.1 1.04 0.88 0.8 0.64 0.38 0.1]];

% fit the input angle of attack into the interval [-pi,pi]
if attack > pi
    attack = mod(attack+pi,2*pi) - pi;
else
    if attack < -pi
        attack = mod(attack-pi,-2*pi) + pi;
    end
end

% interpolation
attack = attack/pi*180;
Cls = interp1(xdata,yldata,attack,'pchip');
Cds = interp1(xdata,yddata,attack,'pchip');
end