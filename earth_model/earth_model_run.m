clear all
close all
clc 

format long 

% UTC Time
UTC     = datetime(2000,3,21,6,0,0);                                      % [time] UTC Time
JD_UTC  = jd(UTC.Year,UTC.Month,UTC.Day,UTC.Hour,UTC.Minute,UTC.Second);   % [day]
T_UTC   = (JD_UTC -2451545.0 )/36525;                                      % [century]
d_AT    = 33.0;        % [sec] From Astronomical Almanac 2006:K9
d_UT1   = 0.2653628;   % [sec]


[UT1, T_UT1, JD_UT1, UTC, TAI, TT, T_TT, JD_TT, T_TDB, JD_TDB] ...
    = convtime (UTC.Year,UTC.Month,UTC.Day,UTC.Hour,UTC.Minute,UTC.Second,d_UT1, d_AT);

% GST Time
lon  = -0;                                                            % [deg]
lat  = 10;

GST = -6.2e-6 * (T_UT1)^3 + 0.093104 * (T_UT1)^2  ...
              + (876600.0 * 3600.0 + 8640184.812866) * T_UT1 + 67310.54841;  % [arcsec]

GST = rem( GST,86400 );  % [arcsec] Remainder of secs 
GST = arcsec2deg(GST)    % [deg] 
GST = degwrap(GST)

LST = deg2rad(GST + lon)
GST = deg2rad(GST)
LAT = deg2rad(lat);

R_E_I = dcm(3,GST)
R_L_I = dcm(2,-LAT)*dcm(3,LST)

%% SUN


fprintf('UTC time: %s\n', datestr(UTC));

% FOR COMPARISON WITH BST MODEL
FROM     = datetime(1970,1,1,0,0,0);                                            % [time]
JD_FROM  = jd(FROM.Year,FROM.Month,FROM.Day,FROM.Hour,FROM.Minute,FROM.Second); % [day]
JD_FROM  = JD_FROM*86400                                                      ; % [sec]

fprintf('UTC time: %.6f\n', JD_FROM);

% INITIAL PARAMETERS

x_p = 0.103267 * arcsec2rad;  % [rad]
y_p = 0.373786 * arcsec2rad;  % [rad]

lod   = 0.0009307;
ddPsi = -0.55418 * arcsec2rad;   % [rad]
ddEps = -0.005137 * arcsec2rad;  % [rad]


[r_sun_mod,rtasc_sun,decl_sun] = sun (JD_UTC);

fprintf('Sun Vector (MOD): %.12f %.12f %.12f\n',r_sun_mod);
        
%% ECI
vmod = [0; 0; 0];
amod = [0; 0; 0];
[r_sun_eci,v_sun_eci,a_sun_eci] = mod2eci(r_sun_mod,vmod,amod,T_TT );

fprintf('Sun Vector (ECI): %.12f %.12f %.12f\n',r_sun_eci);

%% SIMULATION
% Figure Setting
fig = figure;
screensize = get(0,'ScreenSize');
set(fig,'Position',[screensize(3) 0 screensize(4)*0.8 screensize(4)*1]);
grid on; axis fill;
cameratoolbar('SetMode','orbit')   
set(gca,'Position',[0 0 1 1]); % Set Position of Graph
set(gca,'CameraViewAngle',4);  % Set Zoom of Graph
axis(3.5*[-1 1 -1 1 -1 1]);    % Set Limit of Axis 

% Inertial Frame
X_eci = plotvector([1 ;0 ;0], [0 0 0], 'k', 'X_e_c_i');
Y_eci = plotvector([0 ;1 ;0], [0 0 0], 'k', 'Y_e_c_i');
Z_eci = plotvector([0 ;0 ;1], [0 0 0], 'k', 'Z_e_c_i');
hold on;

% Create Sphere
radius  = 1;
[x,y,z] = sphere(20); 
earth   = surf(radius*x,radius*y,radius*z);
set(earth,'facecolor','none','edgecolor',0.7*[1 1 1],'LineStyle',':'); 

% Sun Frame
S_eci = plotvector(r_sun_eci, [0;0;0], [1 0.75 0], 'Sun_E_C_I',2);
S_mod = plotvector(r_sun_mod, [0;0;0], [1 0 0], 'Sun_M_O_D',2);

% Earth Frame
X_ecf = plotvector(R_E_I'*[1 ;0 ;0], [0 0 0], 'r', 'X_e_c_f');
Y_ecf = plotvector(R_E_I'*[0 ;1 ;0], [0 0 0], 'r', 'Y_e_c_f');
Z_ecf = plotvector(R_E_I'*[0 ;0 ;1], [0 0 0], 'r', 'Z_e_c_f');

%local Point
R_local = plotvector(R_L_I'*[1 ;0 ;0], [0 0 0], 'b', 'local');
