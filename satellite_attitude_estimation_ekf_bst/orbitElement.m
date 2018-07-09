function output = orbitElement(input)
%% MODEL TIME
t = get_param('satellite_attitude_estimation_ekf_bst_model','SimulationTime');

%% GLOBAL PARAMETERS
global CONST
Re    = CONST.Re;        % [m] Earth Radius
gamma = CONST.gamma;     % [rad] Earth spin axis tilt with respect to the ecliptic plane
mu    = CONST.mu;        % [m^3/s^2]
J2    = CONST.J2;        % [-] J2 term
a     = CONST.a;         % [m] semi-major axis   
w_0   = CONST.RAAN;      % [rad] Initial Right Ascention - Angle from the X Axis
u_0   = CONST.u_0;       % [rad] Initial Sun Ascension
u_dot = CONST.OmegaDot;  % [rad/s] Ascending node advance for sun-synch
i     = CONST.i;         % [rad] Orbit Inclination
e     = CONST.e;         % [-] Eccentricity 

%% INPUT
R = input(1:3);  % [m] Position Vector in Inertial Frame
V = input(4:6);  % [m/s] Velocity Vector in Inertial Frame

r = norm(R);       % [m]   Position Magnitude of satellite
v = norm(V);       % [m/s] Velocity Magnitude of Satellite
u = u_0 + u_dot*t; % [rad] Right ascension of the sun in the ecliptic plane

%% RIGHT ASCENSION OF LINE OF NODES 
w_dot   = (-9.93639*((Re/r))^(7/2)*cos(i))/((1-e^2)^2)/(180/pi*(60*60*24));                  % [rad/s] Right ascension rate of ascending ~1.99107E-07 [rad/s] for sun synchronous
rate_RA = -(3/2*sqrt(mu)*J2*Re^2/(1-e^2)^2/a^(7/2))*cos(i);                              % [rad/s] Rate of Right ascension ascending ~1.99107E-07 [rad/s] for sun synchronous
rate_AP = -(3/2*sqrt(mu)*J2*Re^2/(1-e^2)^2/a^(7/2))*(5/2*(sin(i)^2)-2)*180/pi*(60*60*24); % [rad/s] Rate of Argument of Perigee

w = w_0+w_dot*t ;    % [rad] Right ascension of the line of nodes
RA = rate_RA*t ;     % [rad] Right ascension of the line of nodes
AP = rate_AP*t ;     % [rad] Argument of Perigee (not considered)
%% PERIFOCAL FRAME 
H = cross(R,V);                    % [m^2/s] Angular Momentum Vector Z axis of Perifocal Frame
N = cross([0;0;1],H);              % [] Nodal Vector N of Perifocal Frame pointing to asending node

%% SATELLITE TRUE ANOMALY
RP = cross(H,R);                     % [ ] Vector of Tangential Velocity
Vr = dot(V,R)/r*R/r;                 % [m/s] Radial Velocity
Vt = dot(RP,V)/norm(RP)*RP/norm(RP); % [m/s] Tangential Velocity

AP = acos(N(1)/norm(N));             % [] Right Ascension of Ascending Node based on X axis of Perifocal Frame

% E = 1/mu*((v^2-mu/r)*R-dot(R,V)*V); % [-] Eccentricity vector  pointing to Periapsis
E = 1/mu*(cross(V,H)-mu*R/r);         % [-] Eccentricity vector pointing to Periapsis (Wertz 6-9)
E = E/norm(E);                        % [-] Eccentricity vector pointing to Periapsis
Y = cross(H,E);                       % [-] Y vector of Perifocal Frame
TA = acos(dot(E,R)/norm(E)/r);        % [rad] True Anomaly of Orbit 
if acos(dot(Y,R)/norm(Y)/norm(R))>pi/2
    TA = 2*pi-TA;
end

beta = asin(  sin(u)*sin(gamma)*cos(i)...
            + cos(u)*sin(i)*sin(w)...
            - sin(u)*cos(gamma)*sin(i)*cos(w)); % [rad] smallest angle between sun vector(earth to sun) and orbit plane
%% LAT/LONGITUDE DETERMINATION
% Calculate Earth Coordinate
ThetaE = 0;                    % [rad] Angle from Z-Axis of Earth Inertia Frame
PhiE = t*2*pi/(23.93*3600);    % [rad] Angle from X-Axis of Earth Inertia Frame
EarthCoord = [Re,ThetaE,PhiE]; % Earth's Position in Spherical Coordinate

% Calculate Sat Coordinate 
X = R(1); % [m] X-coordinates of Satellite
Y = R(2); % [m] Y-coordinates of Satellite
Z = R(3); % [m] Z-coordinates of Satellite

Theta = atan2(sqrt(X^2+Y^2),Z); % [rad] Angle from Z-Axis of Earth Inertia Frame
Phi   = atan2(Y,X);             % [rad] Angle from X-Axis of Earth Inertia Frame
SatCoord = [r,Theta,Phi];

% Calculate Lat and Long
Lat = pi/2-SatCoord(2);                 % [rad] Latitude is angle from Equatorial Plane 
Lon = (SatCoord(3)-EarthCoord(3));      % [rad] Longitude is angle from X-axis of Earth Centered Fixed Frame
if Lon > pi
Lon = Lon-2*pi;
end
if Lon < -pi
Lon = Lon+2*pi;
end

output(1,1) = TA;
output(2,1) = beta;
output(3:5,1) = Vt;
output(6:8,1) = Vr;
output(9,1)   = u;
output(10,1)  = Lat;
output(11,1)  = Lon;
end