% function [pm] = polarm ( xp, yp, ttt, opt )
%   This function calulates the transformation matrix that accounts for polar
%   motion. both the 1980 and 2000 theories are handled. note that the rotation 
%   order is different between 1980 and 2000 .
%
%  inputs          description                    range / units
%    xp          - polar motion coefficient       rad
%    yp          - polar motion coefficient       rad
%    ttt         - julian centuries of tt (00 theory only)
%    opt         - method option                  '01', '02', '80'
%
%  outputs       :
%    pm          - transformation matrix for ecef - pef
%
%  locals        :
%    convrt      - conversion from arcsec to rad
%    sp          - s prime value
%
%  author        : vallado     - 719-573-2600                  25 jun 2002
%  revisions     : vallado     - consolidate with iau 2000     14 feb 2005
%                  rusty       - initiate                      05 jul 2018
%
%  references    :
%    vallado       2004, 207-209, 211, 223-224

function [pm] = polarm ( xp, yp, ttt, opt )

cosxp = cos(xp);
sinxp = sin(xp);
cosyp = cos(yp);
sinyp = sin(yp);

if (opt == '80')
    
    % IAU 1980 theory
	pm(1,1) =  cosxp;
    pm(1,2) =  0.0;
    pm(1,3) = -sinxp;
    pm(2,1) =  sinxp * sinyp;
    pm(2,2) =  cosyp;
    pm(2,3) =  cosxp * sinyp;
    pm(3,1) =  sinxp * cosyp;
    pm(3,2) = -sinyp;
    pm(3,3) =  cosxp * cosyp;

else
    
    % IAU 2000 theory
    sp = -47.0e-6 * ttt * pi / (3600.0*180.0);
    cossp = cos(sp);
    sinsp = sin(sp);
    
    pm(1,1) =  cosxp * cossp;
    pm(1,2) = -cosyp * sinsp + sinyp * sinxp * cossp;
    pm(1,3) = -sinyp * sinsp - cosyp * sinxp * cossp;
    pm(2,1) =  cosxp * sinsp;
    pm(2,2) =  cosyp * cossp + sinyp * sinxp * sinsp;
    pm(2,3) =  sinyp * cossp - cosyp * sinxp * sinsp;
    pm(3,1) =  sinxp;
    pm(3,2) = -sinyp * cosxp;
    pm(3,3) =  cosyp * cosxp;

end;

