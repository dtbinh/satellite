function output = orbitelement(input)
global CONST
Re     = CONST.Re;        % [m] Earth Radius
u_dot  = CONST.OmegaDot;  % [rad/s] Ascending node advance for sun-synch
gamma  = CONST.gamma;     % [rad] Earth spin axis tilt with respect to the ecliptic plane
mu     = CONST.mu;        % [m^3/s^2]
J2     = CONST.J2;        % [-] J2 term
a      = CONST.a;         % [m] semi-major axis   
w_0    = CONST.RAAN;      % [rad] Initial Right Ascention - Angle from the X Axis
u_0    = CONST.u_0;       % [rad] Initial Sun Ascension
i      = CONST.incl;
e      = CONST.ecc;
%% INPUT
R = input(1:3);% [m] Position Vector in Inertial Frame
V = input(4:6);% [m/s] Velocity Vector in Inertial Frame
t = get_param('satellite_attitude_kalman_ekf_model','SimulationTime');

r = norm(R); % [m]   Position Magnitude of satellite

u = u_0 + u_dot*t;  % [rad] Right ascension of the sun in the ecliptic plane

%% RIGHT ASCENSION OF LINE OF NODES 
w_dot = (-9.93639*((Re/r))^(7/2)*cos(i))/((1-e^2)^2)/(180/pi*(60*60*24));                  % [rad/s] Right ascension rate of ascending ~1.99107E-07 [rad/s] for sun synchronous
rate_RA = -(3/2*sqrt(mu)*J2*Re^2/(1-e^2)^2/a^(7/2))*cos(i);                              % [rad/s] Rate of Right ascension ascending ~1.99107E-07 [rad/s] for sun synchronous
rate_AP = -(3/2*sqrt(mu)*J2*Re^2/(1-e^2)^2/a^(7/2))*(5/2*(sin(i)^2)-2)*180/pi*(60*60*24); % [rad/s] Rate of Argument of Perigee

w = w_0+w_dot*t ;    % [rad] Right ascension of the line of nodes
RA = rate_RA*t ;     % [rad] Right ascension of the line of nodes
AP = rate_AP*t ;     % [rad] Argument of Perigee (not considered)

%% PERIFOCAL FRAME 
H = cross(R,V);                    % [m^2/s] Angular Momentum Vector Z axis of Perifocal Frame
N = cross([0;0;1],H);              % [] Nodal Vector N of Perifocal Frame pointing to asending node

%% SATELLITE TRUE ANOMALY

RP = cross(H,R);                    % [ ] Vector of Tangential Velocity
Vr = dot(V,R)/r*R/r;                % [m/s] Radial Velocity
Vt = dot(RP,V)/norm(RP)*RP/norm(RP);% [m/s] Tangential Velocity

AP = acos(N(1)/norm(N));  % [] Right Ascension of Ascending Node based on X axis of Perifocal Frame

E = 1/mu*(cross(V,H)-mu*R/r);         % [-] Eccentricity vector pointing to Periapsis (Wertz 6-9)
E = E/norm(E);                        % [-] Eccentricity vector pointing to Periapsis
Y = cross(H,E);                       % [-] Y vector of Perifocal Frame
TA = acos(dot(E,R)/norm(E)/r);        % [rad] True Anomaly 
if acos(dot(Y,R)/norm(Y)/norm(R))>pi/2
    TA = 2*pi-TA;
end



beta = asin(  sin(u)*sin(gamma)*cos(i)...
            + cos(u)*sin(i)*sin(w)...
            - sin(u)*cos(gamma)*sin(i)*cos(w)); % [rad] smallest angle between sun vector(earth to sun) and orbit plane
    

output = [TA;beta;Vt;Vr;u];
end