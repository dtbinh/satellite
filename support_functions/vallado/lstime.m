% [lst,gst] = lstime ( lon, jd );
% 
% This function finds the local sidereal time at a given location and time.
%
% inputs          
%    lon         - site longitude (west -)        -2pi to 2pi rad
%    jd          - julian date                    days from 4713 bc
%
% outputs       
%    lst         - local sidereal time            0.0 to 2pi rad
%    gst         - greenwich sidereal time        0.0 to 2pi rad
%
% require      
%    gstime        finds the greenwich sidereal time
%
% author        : vallado     719-573-2600   27 may 2002
% revisions     : rusty       initial        05 jul 2018     
%                
% references    :
%    vallado       2013, page 188

function [lst,gst] = lstime ( lon, jd )

gst = gstime( jd ); % [rad]
lst = lon + gst;    % [rad]

lst = rem( lst,2.0*pi );

if ( lst < 0.0 )
    lst= lst + twopi;
end

