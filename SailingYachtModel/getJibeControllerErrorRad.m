function [error] = getJibeControllerErrorRad(currentHeading, desiredHeading, apparentWindAngle)

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

end

