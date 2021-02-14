function [Lat, Lon] = MetersToLatLon(x,y)
    %Convert from UTM (meters) to lat and long
    %Coordinates are in meter starting in bottom left of squares which
    %divide earth into 60 x 60 grid
    %convert is to convert to gps to coordinates
    utmZone = '17 T';
    [latitude, longitude] = utm2deg(x, y, utmZone); 
   
    Lat = latitude;
    Lon = longitude;
end
