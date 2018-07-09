close all
clear all
clc
% SSP1 = [47.1726 31.1325]/180*pi;  % [rad] Start Sub Satellite Point in Lat Lon Coordinates
% SSP2 = [45.3062 30.4989]/180*pi; % [rad] End Sub Satellite Point in Lat Lon Coordinates

% Ground Track
SSP1 = [41.511 29.325]/180*pi;  % [rad] Start Sub Satellite Point in Lat Lon Coordinates
SSP2 = [39.676 28.777]/180*pi; % [rad] End Sub Satellite Point in Lat Lon Coordinates

midSSP1 = SSP1 - SSP2;      % [rad]

latSSP1 = SSP1(1);
lonSSP1 = SSP1(2);

latSSP2 = SSP2(1);
lonSSP2 = SSP2(2);

dlatSSP = (SSP2(1)-SSP1(1)); % [rad]
dlonSSP = (SSP2(2)-SSP1(2)); % [rad]

dA = atan2(tan(dlatSSP),sin(dlonSSP)); % [rad]

A = pi/2-abs(dA);

c = pi/2-SSP1(1);
a = pi/2-SSP2(1);
C = asin(sin(c)*sin(A)/sin(a));

C = pi-C;

Re = 6873e3; % [m] Earth's Radius
h  = 500e3;  % [m] Orbit Altitude

rho = asin(Re/(Re+h));     % [rad]
n   = 20/180*pi;             % [rad]
e   = acos(sin(n)/sin(rho)); % [rad]

lamda = pi/2 - e - n;

D = Re*sin(lamda)/sin(n);

fprintf("------------Satellite Target Latitude/Longitude-----------\n\n");
fprintf("Orbit Height   : %.f[m]\n",h);
fprintf("Sub Satellite 1: %.4f %.4f\n",latSSP1/pi*180,lonSSP1/pi*180);
fprintf("Sub Satellite 2: %.4f %.4f\n",latSSP2/pi*180,lonSSP2/pi*180);
fprintf("Delta Lat Lon  : %.4f %.4f\n",dlatSSP/pi*180,dlonSSP/pi*180);
fprintf("Nadir Angle    : %.2f [deg]\n",n/pi*180);
%% WEST TARGET
if dlatSSP>=0
phi = C - pi/2; 
    else
phi = 3*pi/2+A; 
end

latWTarget2_ = acos(cos(lamda)*sin(latSSP2)+sin(lamda)*cos(latSSP2)*cos(phi));         % [rad]

latWTarget2  = pi/2 - latWTarget2_;
dlonTarget = acos((cos(lamda)-sin(latSSP2)*sin(latWTarget2))/(cos(latSSP2)*cos(latWTarget2)));
if dlatSSP>=0
    lonWTarget2 = lonSSP2-dlonTarget;
else
    lonWTarget2 = lonSSP2+dlonTarget;
end
latWTarget1 = latWTarget2 - dlatSSP;
lonWTarget1 = lonWTarget2 - dlonSSP;

fprintf("\n\n-----------West Target---------\n");
fprintf("Azimuth   : %.2f[deg]\n",phi/pi*180);
fprintf("Target 1: %.4f %.4f\n",latWTarget1/pi*180,lonWTarget1/pi*180);
fprintf("Target 2: %.4f %.4f\n",latWTarget2/pi*180,lonWTarget2/pi*180);
%% EAST TARGET
if dlatSSP>=0
phi = C - pi/2 + pi; 
else
phi = A + pi/2;  % Azimuth of TP2 from
end

latETarget2_ = acos(cos(lamda)*sin(latSSP2)+sin(lamda)*cos(latSSP2)*cos(phi));         % [rad]
latETarget2  = pi/2 - latETarget2_;

dlonETarget = acos((cos(lamda)-sin(latSSP2)*sin(latETarget2))/(cos(latSSP2)*cos(latETarget2)));
if dlatSSP>=0
    lonETarget2 = lonSSP2+dlonETarget;
else
    lonETarget2 = lonSSP2-dlonETarget;
end

latETarget1 = latETarget2 - dlatSSP;
lonETarget1 = lonETarget2 - dlonSSP;

fprintf("-----------East Target---------\n");
fprintf("Azimuth   : %.2f[deg]\n",phi/pi*180);
fprintf("Target 1: %.4f %.4f\n",latETarget1/pi*180,lonETarget1/pi*180);
fprintf("Target 2: %.4f %.4f\n",latETarget2/pi*180,lonETarget2/pi*180);

%% WORLD MAP PLOT

ax = worldmap([35 45],[20 35]);
screensize = get(0,'ScreenSize');
% load coastlines
% plotm(coastlat,coastlon)

land = shaperead('landareas', 'UseGeoCoords', true);
geoshow(ax, land, 'FaceColor', [0.5 0.7 0.5])

plotm([latSSP1 latSSP2]/pi*180,[lonSSP1 lonSSP2]/pi*180,'kx-')
textm(latSSP1/pi*180,lonSSP1/pi*180,'SSP1')
textm(latSSP2/pi*180,lonSSP2/pi*180,'SSP2')


plotm([latWTarget1 latWTarget2]/pi*180,[lonWTarget1 lonWTarget2]/pi*180,'bo-')
textm(latWTarget1/pi*180,lonWTarget1/pi*180,'WT1','Color','blue')
textm(latWTarget2/pi*180,lonWTarget2/pi*180,'WT2','Color','blue')

plotm([latETarget1 latETarget2]/pi*180,[lonETarget1 lonETarget2]/pi*180,'ro-')
textm(latETarget1/pi*180,lonETarget1/pi*180,'ET1','Color','red')
textm(latETarget2/pi*180,lonETarget2/pi*180,'ET2','Color','red')

