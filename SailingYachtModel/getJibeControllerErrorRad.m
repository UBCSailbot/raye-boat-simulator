function [error] = getJibeControllerErrorRad(currentHeading, desiredHeading, jibeDirection)

%bind the heading between 0 and 2pi
currentHeading = mod(currentHeading, 2 * pi);
desiredHeading = mod(desiredHeading, 2 * pi);

    %%takes in the direction the boat should turn and get the difference
    %%between current and desired heading in that direction
    if(jibeDirection > 0)
        error = max(currentHeading - desiredHeading, desiredHeading - currentHeading);
    elseif(jibeDirection < 0)
        error = min(currentHeading - desiredHeading, desiredHeading - currentHeading);
    end

end

