close all
clear all
clc

%% INITIAL POINTS
% Point 1
lat1 = -42/180*pi; 
lon1 = 30/180*pi;

% Point 2
lat2 = 25/180*pi;
lon2 = 273/180*pi;

%% MAIN
% Absolute Difference
dlat = lat2 - lat1;
dlon = lon2 - lon1;

% Arc Distance
a    = (sin(dlat/2))^2+cos(lat1)*cos(lat2)*sin(dlon/2)^2;
c    = 2*atan2(sqrt(a),sqrt(1-a))
Re   = 6378000;        % [m] Earth's Radius
dist = Re*c            % [m] Arc Distance from Point 1 to 2

% Bearing
theta = atan2(sin(dlon)*cos(lat2),cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(dlon))

f = 1  ;          % [ ] Fraction between Point 1 and 2 reference to Point 1
d = f*dist          % [m] Distance of the Fraction from Point 1
delta = d/Re        % [rad] Angular Distance between Point 1 and Point 3


a = sin((1-f)*delta)/sin(delta)
b = sin(f*delta)/sin(delta)
x = a*cos(lat1)*cos(lon1) + b*cos(lat2)*cos(lon2)
y = a*cos(lat1)*sin(lon1)+b*cos(lat2)*sin(lon2)
z = a*sin(lat1) + b*sin(lat2)


%% INTERPOLATED POINT
lat3 = asin(sin(lat1)*cos(delta)+cos(lat1)*sin(delta)*cos(theta));
lon3 = lon1 + atan2(sin(theta)*sin(delta)*cos(lat1),cos(delta)-sin(lat1)*sin(lat2));

%lat3  = atan2(z,sqrt(x^2+y^2))
%lon3  = atan2(y,x)

%% OUTPUT
fprintf("Input Coordinates");
fprintf("\nPoint   1: %.2f,%.2f [deg]", lat1/pi*180, lon1/pi*180);
fprintf("\n");
fprintf("\nPoint   2: %.2f,%.2f [deg]", lat2/pi*180, lon2/pi*180);
fprintf("\n");
fprintf("\nDistance:   %.5f [km]", dist/1000);
fprintf("\nAngle:   %.5f [deg]", delta/pi*180);
fprintf("\nBearing: %.5f [deg]", theta/pi*180);

fprintf("\nPercent: %.2f [%%]", f*100);
fprintf("\n");
fprintf("\nPoint   3: %.2f,%.2f [deg]", lat3/pi*180, lon3/pi*180);
fprintf("\n");

%% REFERENCE
% https://www.movable-type.co.uk/scripts/latlong.html
