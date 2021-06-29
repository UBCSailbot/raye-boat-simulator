function [error] = getJibeControllerErrorRad(currentHeading, desiredHeading, apparentWindAngle)

%bound all values between 0 and 2Pi
apparentWindAngle = mod(apparentWindAngle, 2 * pi);
currentHeading = mod(currentHeading, 2 * pi);
desiredHeading = mod(desiredHeading, 2 * pi);

%find heading relative to wind
windRelativeCurrentHeading = mod(currentHeading - apparentWindAngle, 2 * pi);
windRelativeDesiredHeading = mod(desiredHeading - apparentWindAngle, 2 * pi);

%get error term
error = windRelativeDesiredHeading - windRelativeCurrentHeading;

end

