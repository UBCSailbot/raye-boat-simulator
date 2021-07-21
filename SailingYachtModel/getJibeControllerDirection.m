function [jibeDirection] = getJibeControllerDirection(currentHeading, desiredHeading, apparentWindAngle)
%function takes in the current, desired, and apparent wind angle, and
%returns the direction the boat should turn

%bound all values between 0 and 2Pi
%flip wind direction by half a turn so 0 degrees heading relative to wind
%is upwind
apparentWindAngle = mod(apparentWindAngle + pi, 2 * pi);
currentHeading = mod(currentHeading, 2 * pi);
desiredHeading = mod(desiredHeading, 2 * pi);

%find heading relative to wind, where 0 degrees is upwind 
windRelativeCurrentHeading = mod(currentHeading - apparentWindAngle, 2 * pi);
windRelativeDesiredHeading = mod(desiredHeading - apparentWindAngle, 2 * pi);

%find error term
error = windRelativeDesiredHeading - windRelativeCurrentHeading;

%returns direction as a boolean, 1 is for counterclockwise, -1 for
%clockwise

if (error > 0)
    jibeDirection = 1;
elseif (error < 0)
    jibeDirection = -1;  
end

end

