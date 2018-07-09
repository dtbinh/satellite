function [deg,min,sec] = rad2dms(rad)
% This function convert angles expressed in radian to a 
% degree-arcminute-arcsecond (DMS) format 

temp = rad*180/pi; 

deg = fix(temp);
min = fix((temp-deg)*60);
sec = (temp-deg-min/60)*3600;

end