%  [r,v] = coe2rv ( p,ecc,incl,omega,argp,nu);
%  ------------------------------------------------------------ 
%  This function finds the position and velocity vectors in geocentric
%  equatorial (ijk) system given the classical orbit elements.
%
%  inputs          description                    range / units
%    a           - semimajor axis                 m
%    ecc         - eccentricity
%    incl        - inclination                    0.0  to pi rad
%    omega       - right ascension of asc node    0.0  to 2pi rad
%    argp        - argument of perigee            0.0  to 2pi rad
%    nu          - true anomaly                   0.0  to 2pi rad
%    arglat      - argument of latitude      (ci) 0.0  to 2pi rad
%    truelon     - true longitude            (ce) 0.0  to 2pi rad
%    lonper      - longitude of periapsis    (ee) 0.0  to 2pi rad
%
%  outputs       :
%    r           - ijk position vector            m
%    v           - ijk velocity vector            m / s
%
%  locals        :
%    temp        - temporary real*8 value
%    rpqw        - pqw position vector            m
%    vpqw        - pqw velocity vector            m / s
%    sinnu       - sine of nu
%    cosnu       - cosine of nu
%    tempvec     - pqw velocity vector
%
%  coupling      :
%    mag         - magnitude of a vector
%    rot3        - rotation about the 3rd axis
%    rot1        - rotation about the 1st axis
%
%  references    :
%    curtis        2010
%    vallado       2007, 126, alg 10, ex 2-5
%
%  author        :  
%    rusty       - initiate               08 jul 2018

function [r,v] = coe2rv(a,ecc,incl,omega,argp,nu,opt)

mu = 398.6004118e12;      % [m3/s2]

switch opt
    case 'curtis'
        % Angular momentum
        h  = sqrt(a*mu*(1-ecc^2));  % [m2/s] 

        % Perifocal Frame (pqw)
        rpqw = (h^2/mu) * (1/(1 + ecc*cos(nu))) * (cos(nu)*[1;0;0] + sin(nu)*[0;1;0]);
        vpqw = (mu/h) * (-sin(nu)*[1;0;0] + (ecc + cos(nu))*[0;1;0]);

        % Inertial Frame (ijk)
        r = dcm(3,-omega)*dcm(1,-incl)*dcm(3,-argp)*rpqw;
        v = dcm(3,-omega)*dcm(1,-incl)*dcm(3,-argp)*vpqw;
 
    case 'vallado'
        % Constants
        small = 1.0e-10;
 
        % Semilatus Rectum
        p = a*(1-ecc^2);

        % Perifocal Frame (pqw)
        cosnu = cos(nu);
        sinnu = sin(nu);
        temp  = p / (1.0  + ecc*cosnu);
        rpqw= [temp*cosnu;
               temp*sinnu;
                 0.0];

        if ( abs(p) < 0.0001)
            p= 0.0001;
        end
        vpqw  = [-sinnu*sqrt(mu)  / sqrt(p);
               (ecc + cosnu)*sqrt(mu) / sqrt(p);
                            0.0];

        % Inertial Frame (ijk)
        r = dcm(3,-omega )* dcm(1,-incl)*dcm(3,-argp)* rpqw;
        v = dcm(3,-omega )* dcm(1,-incl)*dcm(3,-argp)* vpqw;
    otherwise
end

