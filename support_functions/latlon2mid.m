% function mid = latlon2mid(latlon1, latlon2)
%  -----------------------------------------------------------------------
% This function outputs the mid point between two points defined by latitude
% and longitude based on the great-circle of a unit sphere
% 
% ------------------------------------------------------------------------
function mid = latlon2mid(latlon1, latlon2)
    latlon1 = deg2rad(latlon1);
    latlon2 = deg2rad(latlon2);
    dlon = latlon2(2) - latlon1(2);
    
    
    Bx = cos(latlon2(1))*cos(dlon);
    By = cos(latlon2(1))*sin(dlon);
    
    lat = atan2( sin(latlon1(1))+sin(latlon2(1)) , sqrt((cos(latlon1(1))+Bx)^2 + By^2) );
    lon = latlon1(2) + atan2( By , cos(latlon1(1)) + Bx);
    
    mid = rad2deg([lat;lon]);
    
end