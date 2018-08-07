%  function [lst,gst] = lstime ( lon, jd )
% -------------------------------------------------------------------------
%  This function finds the local sidereal time at a given location.
% 
%  inputs                             
%    lon         - site longitude (west -)        -2pi to 2pi rad
%    jd          - julian date                    days from 4713 bc
%
%  outputs       
%    lst         -  local sidereal time            0.0 to 2pi rad
%    gst         -  greenwich sidereal time        0.0 to 2pi rad
%
%  revisions
%    vallado     -  initiate                  27 may 2002
%    rusty       -  initiate                  07 aug 2018
%
%  references    :
%    vallado       2007, 194, alg 15, ex 3-5
%
% -----------------------------------------------------------------------------

function [lst,gst] = lstime ( lon, jd )

        twopi  = 2.0*pi;

        % ------------------------  implementation   ------------------
        [gst] = gstime( jd );
        lst = lon + gst;

        % ----------------------- check quadrants ---------------------
        lst = rem( lst,twopi );
        if ( lst < 0.0 )
            lst= lst + twopi;
          end

