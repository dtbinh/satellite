clear all

% FROM
FROM     = datetime(1970,1,1,0,0,0); % UTC Time
JD_FROM  = jd(FROM.Year,FROM.Month,FROM.Day,FROM.Hour,FROM.Minute,FROM.Second); % [days]

% UTC Time
UTC     = datetime(2006,4,1,23,58,54.816); % UTC Time
JD_UTC  = jd(UTC.Year,UTC.Month,UTC.Day,UTC.Hour,UTC.Minute,UTC.Second); % [days]
T_UTC   = (JD_UTC -2451545.0 )/36525;

d_AT    = 33.0;        % [sec] From Astronomical Almanac 2006:K9
d_UT1   = 0.2653628;   % [sec]

x_p = 0.103267 * arcsec2rad;  % [rad]
y_p = 0.373786 * arcsec2rad;  % [rad]

lod   = 0.0009307;
ddPsi = -0.55418 * arcsec2rad;   % [rad]
ddEps = -0.005137 * arcsec2rad;  % [rad]

[UT1, T_UT1, JD_UT1, UTC, TAI, TT, T_TT, JD_TT, T_TDB, JD_TDB] ...
         = convtime (UTC.Year,UTC.Month,UTC.Day,UTC.Hour,UTC.Minute,UTC.Second,d_UT1, d_AT);

% Assumptions
T_TDB  = T_UTC;
JD_TDB = JD_UTC;
T_UT1  = T_UTC;

% Mean Longitude of Sun in MOD frame
% mean_lon_sun = 280.460 + 36000.771*T_UT1;          % [deg]
mean_lon_sun = 280.460 + 36000.77*T_UT1;          % [deg]
mean_lon_sun = rem(mean_lon_sun,360);               % [deg]
mean_lon_sun

% Mean Anomaly of the Sun
% mean_anomaly_sun = 357.5291092 + 35999.05034*T_TDB   % [deg]
mean_anomaly_sun = 357.5277233 + 35999.05034*T_TDB;    % [deg]
mean_anomaly_sun = rem( mean_anomaly_sun,360 );        % [deg]
if ( mean_anomaly_sun < 0.0  )
    mean_anomaly_sun = mean_anomaly_sun+360;           % [deg]
end
mean_anomaly_sun
mean_anomaly_sun = mean_anomaly_sun *deg2rad;          % [rad]


% Ecliptic Longitude 
eclp_lon = mean_lon_sun + 1.914666471*sin(mean_anomaly_sun)...
            + 0.019994643*sin(2.0*mean_anomaly_sun);   % [deg]
eclp_lon = rem(eclp_lon,360.0);                       % [deg]
eclp_lon
eclp_lon = eclp_lon*deg2rad;                        % [rad]

% Ecliptic Latitude 
eclp_lat = 0;                                                         % [deg]

% Obliquity of Ecliptic 
obl_eclp = 23.439291-0.0130042*T_TDB;            % [deg] obl_eclp of Ecliptic
obl_eclp
obl_eclp = obl_eclp*deg2rad;                     % [rad]

% Eccentricuty of Earth's Orbit
ecc_earth = 0.016708617 - 0.000042037*T_TDB-0.0000001236*T_TDB^2 % [-] Eccentricity of Earth's Orbit

% Sun's Vector
R_sun  = 1.000140612-0.016708617*cos(mean_anomaly_sun)...
        -0.000139589*cos(2.0*mean_anomaly_sun)      % [AU]
r_sun  = [  R_sun*cos(eclp_lon);                    % [AU]
            R_sun*cos(obl_eclp)*sin(eclp_lon);      % [AU]
            R_sun*sin(obl_eclp)*sin(eclp_lon)]      % [AU]

r_sun_km = r_sun*149597870.0                        % [km]

% Right Ascension of the Sun
rtasc_sun = atan2( cos(obl_eclp)*tan(eclp_lon),1 )

% --- check that rtasc_sun is in the same quadrant as eclp_lon ----
        if ( eclp_lon < 0.0  )
            eclp_lon= eclp_lon + twopi;    % make sure it's in 0 to 2pi range
        end
        if ( abs( eclp_lon-rtasc_sun ) > pi*0.5  )
            rtasc_sun= rtasc_sun + 0.5 *pi*round( (eclp_lon-rtasc_sun)/(0.5 *pi));
        end
        
% Declination of Sun        
decl_sun = asin( sin(obl_eclp)*sin(eclp_lon) );
decl_sun

%% ASTRONOMICAL ALM VALUE
r_sun_aa = [0.9775113 ; 0.1911521 ; 0.0828717]*149597870.0 % astronomical alm value into km
        

%% MOD -> ECI

vmod = [0; 0; 0];
amod = [0; 0; 0];
[r_sun_eci,v_sun_eci,a_sun_eci] = mod2eci(r_sun,vmod,amod,T_TT );
r_sun_eci*149597870.0

%% SIMULATION
% Figure Setting
fig = figure;
screensize = get(0,'ScreenSize');
set(fig,'Position',[0 0 screensize(4)*0.8 screensize(4)*0.8]);
grid on; axis fill;
cameratoolbar('SetMode','orbit')   
set(gca,'Position',[0 0 1 1]); % Set Position of Graph
set(gca,'CameraViewAngle',4);  % Set Zoom of Graph
axis(3.5*[-1 1 -1 1 -1 1]);    % Set Limit of Axis 

% Earth Centered Inertial Frame
X_eci = plotvector([1 ;0 ;0], [0 0 0], 'k', 'X_e_c_i');
Y_eci = plotvector([0 ;1 ;0], [0 0 0], 'k', 'Y_e_c_i');
Z_eci = plotvector([0 ;0 ;1], [0 0 0], 'k', 'Z_e_c_i');
hold on;

% Create Earth Sphere
radius  = 1;
[x,y,z] = sphere(20); 
earth   = surf(radius*x,radius*y,radius*z);
set(earth,'facecolor','none','edgecolor',0.7*[1 1 1],'LineStyle',':'); 