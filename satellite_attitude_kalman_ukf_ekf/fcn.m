function output = fcn(input)
% This function computes the classical orbital elements
% from the state vector (R,V) using Algorithm 4.1. As well as
% other orbital parameters needed by the model.
%
% mu - gravitational parameter (mˆ3/sˆ2)
% R - position vector in the geocentric equatorial frame (m)
% V - velocity vector in the geocentric equatorial frame (m/s)
% r, v - the magnitudes of R and V
% vr - radial velocity component (m/s)
% H - the angular momentum vector (mˆ2/s)
% h - the magnitude of H (mˆ2/s)
% incl - inclination of the orbit (rad)
% N - the node line vector (mˆ2/s)
% n - the magnitude of N
% cp - cross product of N and R
% RA - right ascension of the ascending node (rad) not used
% E - eccentricity vector
% ecc - eccentricity (magnitude of E)
% eps - a small number below which the eccentricity is
% considered to be zero
% w - argument of perigee (rad) not used

% TA - true anomaly (rad)
% Vt - tangential velocity (m/s)
% Vr - radial velocity (m/s)
% rho_earth - earth anglular radius
% beta - beat angle (rad)
% Lat - Latitude of satellite (rad)
% Long - Longitude of satellite (rad)
% ------------------------------------------------------------

% time,Re,incl,ecc,R,V
time = input(1,1);
Re   = input(2,1);
incl = input(3,1);
ecc  = input(4,1);
R    = input(5:7,1);
V    = input(8:10,1);

mu = 398.6004418e12; % m^3/s^2
eps = 1.0e-10;
r = norm(R);
v = norm(V);
vr = dot(R,V)/r;
H = cross(R,V);
h = norm(H);
% Calc inclination
%{
c = H(3)/h;
if (c < -1) && (c > 1)
c = c - pi;
end
incl = acos(c);
%}
% Calc right ascension of the ascending node (rad)
N = cross([0 0 1],H);

n = norm(N);
% Calc Eccentricity
E = 1/mu*((v^2 - mu/r)*R - r*vr*V);
%ecc = norm(E);

% True Annomoly
if ecc > eps
    c = dot(E,R)/ecc/r;
        if (c < -1) && (c > 1)
            c = c - pi;
        end
    TA = acos(c);
        if vr < 0
            TA = 2*pi - TA;
        end
else
    cp = cross(N,R);
    c = dot(N,R)/n/r;
    if (c < -1) && (c > 1)
        c = c - pi;
    end
    if cp(3) >= 0
        TA = acos(c);
    else
        TA = 2*pi - acos(c);
    end
end

% Calculate the tangential and radial velocities
Vt = h/r;
Vr = mu/h*ecc*sin(TA);

% Calculate earth angular radius
rho_earth = asin(Re/r);

% Beta calcs
wb_0 = 0;
ub_0 = 0;
wb_dot = (-9.9639/86400)*rho_earth^(3.5)*cos(incl)/(1-ecc^2)^2;
wb = (wb_0 + wb_dot*time)*pi/180;
gamma = 23.442*pi/180; %rad
ub_dot = (0.985648/86400)*pi/180; %rad
ub = ub_0+ub_dot*time;
beta = asin(sin(ub)*sin(gamma)*cos(incl) + ...
cos(ub)*sin(incl)*sin(wb)-sin(ub)*cos(gamma)*sin(incl)*cos(wb));

% For Mag Calc ----------------------------------------------
% Calculate Earth Coordinate by Simulate the Earth's Rotation
% Track the movement of (0 Lat, 0 Long)
PhiE = time*2*pi/(23.93*3600);
ThetaE = 0;
EarthCoord = [Re,ThetaE,PhiE];

% Calculate Sat Coordinate in Polar
X = R(1);
Y = R(2);
Z = R(3);
Theta = atan2(sqrt(X^2+Y^2),Z);
Phi = atan2(Y,X);
SatCoord = [r,Theta,Phi];

% Calculate Lat and Long
Theta0 = pi/2-SatCoord(2);
Phi0 = SatCoord(3);
Phi1 = EarthCoord(3);
Lat = Theta0;
Long = (Phi0-Phi1);
    if Long > pi
        Long = Long-2*pi;
    end
if Long < -pi
    Long = Long+2*pi;
end

% [rho_earth,beta,TA,r,Vt,Vr,Lat,Long]
output(1,1) = rho_earth;
output(2,1) = beta;
output(3,1) = TA;
output(4,1) = r;
output(5,1) = Vt;
output(6,1) = Vr;
output(7,1) = Lat;
output(8,1) = Long;
end