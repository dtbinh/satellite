% function latlon2vec(lat,lon,gst)
% -------------------------------------------------------------------------
% This function converts lat/lon to vector in the inertial frame
% 
% Input
%   lat     -   [deg] latitude in ecef frame
%   lon     -   [deg] longitude in ecef frame
%   gst     -   [deg] greenrich sidereal time
% 
% Output
%    vec    -   [m]   unit vector in inertial frame 
% 
% 

function vec = latlon2vec(lat,lon,gst)
lat = deg2rad(lat);      % [rad] 
lon = deg2rad(lon);      % [rad] 
gst = deg2rad(gst);      % [rad]

lst = gst + lon;         % [rad]

rotm = dcm(2,-lat)*dcm(3,lst);
vec     = rotm'*[1 ;0 ;0];