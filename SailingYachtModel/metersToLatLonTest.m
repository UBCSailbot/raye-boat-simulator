%CN Tower
testFunction(630084,4833438, 43.6426,-79.3871);
testFunction(530444 ,2833438 ,25.6182,-80.6968);

function testFunction(x,y, expectedLat, expectedLon)
    [lat, lon] = metersToLatLon(x,y);
     assert(round(lat,4) == expectedLat && round(lon, 4) == expectedLon);
end 