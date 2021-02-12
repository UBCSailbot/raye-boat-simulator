function res = isAngleBetween(boundAngle1, boundAngle2, checkAngle)
    
    absDiffBetweenBoundAngles = abs(angdiff(boundAngle1,boundAngle2));
    
res = abs(absDiffBetweenBoundAngles - (abs(angdiff(boundAngle1,checkAngle)) + abs(angdiff(boundAngle2,checkAngle))))<0.000001;
end
