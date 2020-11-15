function theta = apparentWind(surge, sway, windSpeed, windAngle, psi)

boatVel = [surge, sway];
windAngle = windAngle - psi;
[windx, windY] = pol2cart(windAngle, windSpeed);
windVec = [windx, windY];
windVec =  windVec - boatVel;
apparentWind = cart2pol(windVec(1), windVec(2));
theta = apparentWind(1);
