function error = getHeadingError(setPoint, measure)

dist1 = setPoint - measure;
dist2 = 2 * pi + dist1;
if(dist2 > 2 * pi)
   dist2 = -(2 * pi - dist1);
end

if(abs(dist1) < abs(dist2))
    error = dist1;
else 
    error = dist2;
end

