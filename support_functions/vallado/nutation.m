function [deltapsi, trueeps, meaneps, omega, nut] = nutation(ttt, ddpsi, ddeps)
%  this function calulates the transformation matrix that accounts for the
%    effects of nutation.
% inputs          description                    range / units
%    ttt         - julian centuries of tt
%    ddpsi       - delta psi correction to gcrf   rad
%    ddeps       - delta eps correction to gcrf   rad
%
% outputs       
%    deltapsi    - nutation angle                 rad
%    trueeps     - true obliquity of the ecliptic rad
%    meaneps     - mean obliquity of the ecliptic rad
%    omega       -                                rad
%    nut         - transformation matrix for tod - mod
%
% locals        
%    iar80       - integers for fk5 1980
%    rar80       - reals for fk5 1980
%    ttt2        - ttt squared
%    ttt3        - ttt cubed
%    l           -                                rad
%    ll          -                                rad
%    f           -                                rad
%    d           -                                rad
%    deltaeps    - change in obliquity            rad
%  references    : vallado       2013, 224-226
%  author        : rusty

deg2rad = pi/180.0;

[iar80,rar80] = iau80in;  % coeff in deg

ttt2 = ttt*ttt;
ttt3 = ttt2*ttt;

meaneps = -46.8150 *ttt - 0.00059 *ttt2 + 0.001813 *ttt3 + 84381.448;
meaneps = rem( meaneps/3600.0, 360.0 );
meaneps = meaneps * deg2rad;
        
[ l, l1, f, d, omega, lonmer, lonven, lonear, lonmar, lonjup, lonsat, lonurn, lonnep, precrate ...
        ] = fundarg( ttt, '80' );
    
% Nutation in Longitude (deltapsi) and Nutation in Obliquity (deltaeps)
deltapsi = 0.0;
deltaeps = 0.0;
    
for i= 106:-1: 1
    tempval= iar80(i,1)*l + iar80(i,2)*l1 + iar80(i,3)*f + ...
             iar80(i,4)*d + iar80(i,5)*omega;
    deltapsi= deltapsi + (rar80(i,1)+rar80(i,2)*ttt) * sin( tempval );
    deltaeps= deltaeps + (rar80(i,3)+rar80(i,4)*ttt) * cos( tempval );
end

deltapsi = rem( deltapsi + ddpsi, 2.0 * pi );
deltaeps = rem( deltaeps + ddeps, 2.0 * pi );
trueeps  = meaneps + deltaeps;

% Nutation Matrix
cospsi  = cos(deltapsi);
sinpsi  = sin(deltapsi);
coseps  = cos(meaneps);
sineps  = sin(meaneps);
costrueeps = cos(trueeps);
sintrueeps = sin(trueeps);

nut(1,1) =  cospsi;
nut(1,2) =  costrueeps * sinpsi;
nut(1,3) =  sintrueeps * sinpsi;
nut(2,1) = -coseps * sinpsi;
nut(2,2) =  costrueeps * coseps * cospsi + sintrueeps * sineps;
nut(2,3) =  sintrueeps * coseps * cospsi - sineps * costrueeps;
nut(3,1) = -sineps * sinpsi;
nut(3,2) =  costrueeps * sineps * cospsi - sintrueeps * coseps;
nut(3,3) =  sintrueeps * sineps * cospsi + costrueeps * coseps;

 end