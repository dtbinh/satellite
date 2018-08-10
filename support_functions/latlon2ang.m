% function ang = latlon2ang(latlon1, latlon2)
%  -----------------------------------------------------------------------
% This function outputs the angular distance between two points defined by latitude
% and longitude based on the great-circle of a unit sphere
% 
% ------------------------------------------------------------------------
function ang = latlon2ang(latlon1, latlon2)

    latlon1 = deg2rad(latlon1);
    latlon2 = deg2rad(latlon2);
    
    dlon = latlon2(2) - latlon1(2);

    % Law of Cosine
    ang = acos(sin(latlon1(1)) * sin(latlon2(1))  +  cos(latlon1(1)) * cos(latlon2(1)) * cos(dlon)) ;
    
end