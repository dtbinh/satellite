% function int = latlon2int(latlon1, latlon2, fract)
%  -----------------------------------------------------------------------
% This function outputs the intermediate point between two points defined by latitude
% and longitude based on the great-circle of a unit sphere
% 
% ------------------------------------------------------------------------
function int = latlon2int(latlon1, latlon2, fract)
    delta = latlon2ang(latlon1,latlon2);
    
    latlon1 = deg2rad(latlon1);
    latlon2 = deg2rad(latlon2);
    
    
    
    a = sin((1-fract)*delta)/sin(delta);
    b = sin(fract*delta)/sin(delta);
    
    x = a*cos(latlon1(1))*cos(latlon1(2))  +  b*cos(latlon2(1))*cos(latlon2(2));
    y = a*cos(latlon1(1))*sin(latlon1(2))  +  b*cos(latlon2(1))*sin(latlon2(2));
    z = a*sin(latlon1(1))    +    b*sin(latlon2(1));
    
      
    
    lat = atan2( z , sqrt(x^2 + y^2));
    lon = atan2( y , x );
    
    int = rad2deg([lat;lon]);
    
end