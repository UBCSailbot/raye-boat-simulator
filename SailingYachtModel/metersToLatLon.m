function [lat, lon] = metersToLatLon(x,y)
    %Convert from UTM (meters) to lat and long
    %Coordinates are in meter starting in bottom left of squares which
    %divide earth into 60 x 60 grid
    %returns GPS coordinates in decimal degrees
    utmZone = '10 N'; %10 N is near Vancouver
    
    %in utm2deg coordinate x is east and y is north. In Jouffrey paper, x
    %is north y is east. so switching the input.
    
    %calling y,x because in Jouffrey paper, x is north and y is east
    [lat, lon] = utm2deg(y, x, utmZone); 
   
end