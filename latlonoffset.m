% function latlonoffset(tgt_start, tgt_end, angle, orbit)
% ------------------------------------------------------------------------
% This function calculates the latitude and longitude from a set of
% latitude and longitude based on an offset angle from the satellite
% position
%
% Input:
% 
% 
% 
% Output:
% 
% 
% 
function [tgt_west,tgt_east] = latlonoffset(tgt_start, tgt_end, angle, orbit)
% Ground Track 
SSP1 = [tgt_start.lat tgt_start.lon]/180*pi;  % [rad] Start Sub Satellite Point in Lat Lon Coordinates
SSP2 = [  tgt_end.lat   tgt_end.lon    ]/180*pi;  % [rad] End Sub Satellite Point in Lat Lon Coordinates

% Offset Angle
n  = angle/180*pi;             % [rad] Input Offset Angle to point from Nadir

% Orbit Parameter
Re = orbit.Re; % [m] Earth's Radius
h  = orbit.h;  % [m] Orbit Altitude

%% MAIN
midSSP1 = SSP1 - SSP2;          % [rad]

latSSP1 = SSP1(1);             % [rad]
lonSSP1 = SSP1(2);             % [rad]

latSSP2 = SSP2(1);            % [rad]
lonSSP2 = SSP2(2);            % [rad]

dlatSSP = (SSP2(1)-SSP1(1)); % [rad]
dlonSSP = (SSP2(2)-SSP1(2)); % [rad]

dA = atan2(tan(dlatSSP),sin(dlonSSP)); % [rad] dAzimuth

A = pi/2-abs(dA);                      % [rad] Azimuth

c = pi/2-SSP1(1);
a = pi/2-SSP2(1);
C = asin(sin(c)*sin(A)/sin(a));

C = pi-C;


rho = asin(Re/(Re+h));       % [rad]
e   = acos(sin(n)/sin(rho)); % [rad]

lamda = pi/2 - e - n;

D = Re*sin(lamda)/sin(n);
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

tgt_west.endlat   = rad2deg(latWTarget2);             % [deg] 
tgt_west.endlon   = rad2deg(lonWTarget2);             % [deg] 
tgt_west.startlat = rad2deg(latWTarget2 - dlatSSP);   % [deg]     
tgt_west.startlon = rad2deg(lonWTarget2 - dlonSSP);   % [deg]

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
tgt_east.endlat = rad2deg(latETarget2);            % [deg] 
tgt_east.endlon = rad2deg(lonETarget2);            % [deg]
tgt_east.startlat = (latETarget2 - dlatSSP)/pi*180;  % [deg]
tgt_east.startlon = (lonETarget2 - dlonSSP)/pi*180;  % [deg]


end