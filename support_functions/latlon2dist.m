% function dist = latlon2dist(latlon1, latlon2)
%  -----------------------------------------------------------------------
% This function outputs the distance between two poitns defined by latitude
% and longitude based on the great-circle of a unit sphere
% 
% ------------------------------------------------------------------------
function dist = latlon2dist(latlon1, latlon2)
    latlon1 = deg2rad(latlon1);
    latlon2 = deg2rad(latlon2);

    dlat = latlon2(1) - latlon1(1);
    dlon = latlon2(2) - latlon1(2);
    
    a = (sin(dlat/2))^2 + cos(latlon1(1))*cos(latlon2(1))*(sin(dlon/2))^2;
    dist = 2*atan2(sqrt(a),sqrt(1-a));    
    
end