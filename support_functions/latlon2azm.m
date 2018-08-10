% function azm = latlon2azm(latlon1, latlon2)
%  -----------------------------------------------------------------------
% This function outputs the azimuth from point 1 to point 2 between two points defined by latitude
% and longitude based on the great-circle of a unit sphere
% 
% ------------------------------------------------------------------------
function azm = latlon2azm(latlon1, latlon2)
    
    latlon1 = deg2rad(latlon1);
    latlon2 = deg2rad(latlon2);
    
    dlon = latlon2(2) - latlon1(2);
    
    azm = atan2(sin(dlon)*cos(latlon2(1)) ,  cos(latlon1(1))*sin(latlon2(1))- sin(latlon1(1))*cos(dlon));
    
end