function [phi, theta, r] = cart2sphe(x, y, z)

r = sqrt(x.^2+y.^2+z.^2);

% Elevation
theta = acos(z./r)*180/pi;

% Azimuth
phi = atan2(y,x)*180/pi;

end