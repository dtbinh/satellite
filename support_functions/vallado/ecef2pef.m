% [rpef,vpef,apef] = ecef2pef  ( recef,vecef,aecef, xp, yp, ttt )
%
%                    PEF = [polarm]*ITRF
%
%  this function transforms a vector from the earth fixed itrf frame
%    (itrf), to the pseudo earth fixed frame (pef).
%
%  inputs          description                    range / units
%    recef       - position vector earth fixed    km
%    vecef       - velocity vector earth fixed    km/s
%    aecef       - acceleration vector earth fixedkm/s2
%    xp          - polar motion coefficient       arc sec
%    yp          - polar motion coefficient       arc sec
%    ttt         - julian centuries of tt         centuries
%
%  outputs       :
%    rpef        - position pseudo earth fixed    km
%    vpef        - velocity pseudo earth fixed    km/s
%    apef        - acceleration pseudo earth fixedkm/s2
%
%  locals        :
%
%  coupling      :
%   precess      - rotation for precession        mod - eci
%
%  author        : vallado     719-573-2600     27 may 2002
%  revisions     : rusty        -initiate       05 jul 2018
%
%  references    : vallado       2001, 219, eq 3-65 to 3-66
%

function [rpef,vpef,apef] = ecef2pef  ( recef,vecef,aecef, xp, yp, ttt )

        [pm] = polarm(xp,yp,ttt,'80');

        rpef = pm*recef;

        vpef = pm*vecef; 

        apef = pm*aecef;


