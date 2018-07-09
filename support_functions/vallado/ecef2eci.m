% [reci,veci,aeci] = ecef2eci  ( recef,vecef,aecef,ttt,jdut1,lod,xp,yp,eqeterms,ddpsi,ddeps );
%
%  This function transforms a vector from the earth fixed (itrf) frame, to
%  the eci mean equator mean equinox (j2000).
%          
%               GCRF = [precession]*[nutation]*[sidereal]*[polarm]*ITRF
% 
%  inputs          description                    range / units
%    recef       - position vector earth fixed    km
%    vecef       - velocity vector earth fixed    km/s
%    aecef       - acceleration vector earth fixedkm/s2
%    ttt         - julian centuries of tt         centuries
%    jdut1       - julian date of ut1             days from 4713 bc
%    lod         - excess length of day           sec
%    xp          - polar motion coefficient       rad
%    yp          - polar motion coefficient       rad
%    eqeterms    - terms for ast calculation      0,2
%    ddpsi       - delta psi correction to gcrf   rad
%    ddeps       - delta eps correction to gcrf   rad
%
%  outputs       :
%    reci        - position vector eci            km
%    veci        - velocity vector eci            km/s
%    aeci        - acceleration vector eci        km/s2
%
%  locals        :
%    deltapsi    - nutation angle                 rad
%    trueeps     - true obliquity of the ecliptic rad
%    meaneps     - mean obliquity of the ecliptic rad
%    omega       -                                rad
%    prec        - matrix for mod - eci 
%    nut         - matrix for tod - mod 
%    st          - matrix for pef - tod 
%    stdot       - matrix for pef - tod rate
%    pm          - matrix for ecef - pef 
%
%  coupling      :
%   precess      - rotation for precession       
%   nutation     - rotation for nutation          
%   sidereal     - rotation for sidereal time     
%   polarm       - rotation for polar motion      
%
%  author        : vallado                  719-573-2600        4 jun 2002
%
%  revisions     : vallado     - add terms for ast calculation  30 sep 2002
%                  vallado     - consolidate with iau 2000      14 feb 2005
%                  rusty       - initiate                       05 jul 2018
%  references    : vallado       2013, 223-229
%

function [reci,veci,aeci] = ecef2eci  ( recef,vecef,aecef,ttt,jdut1,lod,xp,yp,eqeterms,ddpsi,ddeps );

        % Precess Matrix
        [prec,psia,wa,ea,xa] = precess ( ttt, '80' );

        % Nutation Matrix
        [deltapsi,trueeps,meaneps,omega,nut] = nutation(ttt,ddpsi,ddeps);

        % Sideral Matrix
        [st,stdot] = sidereal(jdut1,deltapsi,meaneps,omega,lod,eqeterms );

        % Polar Effect Matrix
        [pm] = polarm(xp,yp,ttt,'80');

        % Transformations
        thetasa= 7.29211514670698e-05 * (1.0  - lod/86400.0 );
        omegaearth = [0; 0; thetasa;];

        rpef = pm*recef;          % Polar Effect Frame (PEF) 
        reci = prec*nut*st*rpef;  % Geocentric Celestial Reference Frame (GCRF)

        vpef = pm*vecef;
        veci = prec*nut*st*(vpef + cross(omegaearth,rpef));

        aeci = prec*nut*st*( pm*aecef + cross(omegaearth,cross(omegaearth,rpef)) ...
               + 2.0*cross(omegaearth,vpef) );

