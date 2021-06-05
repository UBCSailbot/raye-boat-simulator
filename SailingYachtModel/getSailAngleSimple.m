function [sailAngle] = getSailAngleSimple(windAngle)
%Simplified sail controller, approximates sail angle with basic linear
%function
%lower bound of stepwise function
x1 = 0.25 * 3.14;
%upperbound of stepwise function
x2 = (0.75) * 3.14;

%minimum sail angle
min = 0;

%maximum sail Angle
max = 3.14/2;

if(abs(windAngle) > x2)
   sailAngle = min; 
   
elseif(abs(windAngle) <x1)
    sailAngle = max;
else
    sailAngle = interp1([x1 x2], [max min], abs(windAngle));
end

end

