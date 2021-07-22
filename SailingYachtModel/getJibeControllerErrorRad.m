function [error] = getJibeControllerErrorRad(currentHeading, desiredHeading, jibeDirection)

%bind the heading between 0 and 2pi
currentHeading = mod(currentHeading, 2 * pi);
desiredHeading = mod(desiredHeading, 2 * pi);

    %%takes in the direction the boat should turn and get the difference
    %%between current and desired heading in that direction
    errorOneWay = desiredHeading - currentHeading;
    if(jibeDirection > 0)
        error = max((2 * pi  - abs(errorOneWay)) * -sign(errorOneWay), errorOneWay);
    elseif(jibeDirection < 0)
        error = min((2 * pi  - abs(errorOneWay)) * -sign(errorOneWay), errorOneWay);
    end

end

