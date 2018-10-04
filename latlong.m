close all
clear all
clc
%% INPUT
% Ground Track 
SSP1 = [40.9801358476115 5.40069039583235]/180*pi;  % [rad] Start Sub Satellite Point in Lat Lon Coordinates
SSP2 = [39.1315311094753 4.85221628962299]/180*pi;  % [rad] End Sub Satellite Point in Lat Lon Coordinates

% Offset Angle
n    = deg2rad(10);             % [rad] Input Offset Angle to point from Nadir

% Orbit Parameter
Re = 6873e3; % [m] Earth's Radius
h  = 550e3;  % [m] Orbit Altitude
%% FUNCTION
tgt_start.lat = SSP1(1)/pi*180;
tgt_start.lon = SSP1(2)/pi*180;
tgt_end.lat   = SSP2(1)/pi*180;
tgt_end.lon   = SSP2(2)/pi*180;

angle = rad2deg(n);
orbit.Re = Re;
orbit.h  = h;
[tgt_west,tgt_east] = latlonoffset(tgt_start, tgt_end, angle, orbit);

% %% MAIN
% midSSP1 = SSP1 - SSP2;          % [rad]
% 
% latSSP1 = SSP1(1);
% lonSSP1 = SSP1(2);
% 
% latSSP2 = SSP2(1);
% lonSSP2 = SSP2(2);
% 
% dlatSSP = (SSP2(1)-SSP1(1)); % [rad]
% dlonSSP = (SSP2(2)-SSP1(2)); % [rad]
% 
% dA = atan2(tan(dlatSSP),sin(dlonSSP)); % [rad] dAzimuth
% 
% A = pi/2-abs(dA);                      % [rad] Azimuth
% 
% c = pi/2-SSP1(1);
% a = pi/2-SSP2(1);
% C = asin(sin(c)*sin(A)/sin(a));
% 
% C = pi-C;
% 
% 
% rho = asin(Re/(Re+h));       % [rad]
% e   = acos(sin(n)/sin(rho)); % [rad]
% 
% lamda = pi/2 - e - n;
% 
% D = Re*sin(lamda)/sin(n);

fprintf('------------Satellite Target Latitude/Longitude-----------\n\n');
fprintf('Orbit Height   : %.f[m]\n',h);
fprintf('Sub Satellite 1: %15.12f %15.12f\n',tgt_start.lat,tgt_start.lon);
fprintf('Sub Satellite 2: %15.12f %15.12f\n',tgt_end.lat,tgt_end.lon);
fprintf('Offset Angle    : %.2f [deg]\n',angle);

%% WEST TARGET
% if dlatSSP>=0
% phi = C - pi/2;
%     else
% phi = 3*pi/2+A; 
% end
% 
% latWTarget2_ = acos(cos(lamda)*sin(latSSP2)+sin(lamda)*cos(latSSP2)*cos(phi));         % [rad]
% 
% latWTarget2  = pi/2 - latWTarget2_;
% dlonTarget = acos((cos(lamda)-sin(latSSP2)*sin(latWTarget2))/(cos(latSSP2)*cos(latWTarget2)));
% if dlatSSP>=0
%     lonWTarget2 = lonSSP2-dlonTarget;
% else
%     lonWTarget2 = lonSSP2+dlonTarget;
% end
% latWTarget1 = latWTarget2 - dlatSSP;
% lonWTarget1 = lonWTarget2 - dlonSSP;

fprintf('\n\n-----------West Target---------\n');
% fprintf('Target 1: %15.12f %15.12f\n',latWTarget1/pi*180,lonWTarget1/pi*180);
% fprintf('Target 2: %15.12f %15.12f\n',latWTarget2/pi*180,lonWTarget2/pi*180);
fprintf('Start: %15.12f %15.12f\n',tgt_west.startlat,tgt_west.startlon);
fprintf('End  : %15.12f %15.12f\n',tgt_west.endlat,tgt_west.endlon);
%% EAST TARGET
% if dlatSSP>=0
% phi = C - pi/2 + pi; 
% else
% phi = A + pi/2;  % Azimuth of TP2 from
% end
% 
% latETarget2_ = acos(cos(lamda)*sin(latSSP2)+sin(lamda)*cos(latSSP2)*cos(phi));         % [rad]
% latETarget2  = pi/2 - latETarget2_;
% 
% dlonETarget = acos((cos(lamda)-sin(latSSP2)*sin(latETarget2))/(cos(latSSP2)*cos(latETarget2)));
% if dlatSSP>=0
%     lonETarget2 = lonSSP2+dlonETarget;
% else
%     lonETarget2 = lonSSP2-dlonETarget;
% end
% 
% latETarget1 = latETarget2 - dlatSSP;
% lonETarget1 = lonETarget2 - dlonSSP;

fprintf('-----------East Target---------\n');
% fprintf('Target 1: %15.12f %15.12f\n',latETarget1/pi*180,lonETarget1/pi*180);
% fprintf('Target 2: %15.12f %15.12f\n',latETarget2/pi*180,lonETarget2/pi*180);
fprintf('Start: %15.12f %15.12f\n',tgt_east.startlat,tgt_east.startlon);
fprintf('End  : %15.12f %15.12f\n',tgt_east.endlat,tgt_east.endlon);
return
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

