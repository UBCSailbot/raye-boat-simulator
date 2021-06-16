function [sailAngle] = getSailAngleRadSimple(apparentWindAngleRad)
%Simplified sail controller, approximates sail angle with basic linear
%function
%Aprroximates Sail Control with a stepwise function, if angle to wind is
%below x1 then sets sail angle to maxSailAngle, if the angle is above x2
%then sets sail angle to minSailAngle, otherwise approximates sail angle
%using a linear interpolations
%Method taken from this paper:
%https://core.ac.uk/download/pdf/79618904.pdf
%lower bound of stepwise function
x1 = 0.25 * pi;
%upperbound of stepwise function
x2 = (0.75) * pi;

%minimum sail angle
minSailAngle = 0;

%maximum sail Angle
maxSailAngle = pi/2;

if(abs(apparentWindAngleRad) > x2)
   sailAngle = minSailAngle; 
   
elseif(abs(apparentWindAngleRad) <x1)
    sailAngle = maxSailAngle;
else
    sailAngle = interp1([x1 x2], [maxSailAngle minSailAngle], abs(apparentWindAngleRad));
end

end

