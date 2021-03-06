clear all
close all
clc 

format long 
Re = 1; % [Re] Earth Radius Unit
global CONST

CONST.sidereal = 23.934469583333332; % [hrs] Hours in a Sidereal Day
CONST.solarday = 24.00000000;        % [hrs] Hours in a Solar Day 

%% TIME
% UTC Time - Universal Time
UTC     = datetime(2000,3,21,6,0,0);                                       % [time] UTC Time
JD_UTC  = jd(UTC.Year,UTC.Month,UTC.Day,UTC.Hour,UTC.Minute,UTC.Second);   % [day]
T_UTC   = (JD_UTC -2451545.0 )/36525;                                      % [century]
d_AT    = 33.0;                                                            % [sec] From Astronomical Almanac 2006:K9
d_UT1   = 0.2653628;                                                       % [sec]
[UT1, T_UT1, JD_UT1, UTC, TAI, TT, T_TT, JD_TT, T_TDB, JD_TDB] ...
    = convtime (UTC.Year,UTC.Month,UTC.Day,UTC.Hour,UTC.Minute,UTC.Second,d_UT1, d_AT);

CONST.JD_UT1 = JD_UT1;
CONST.JD_UTC = JD_UTC;
CONST.T_TT   = T_TT;

%% GST - Greenwich Sidereal Time 
GST   = jdut2gst(JD_UT1);     % [rad] 

%% LATLON POINTS
% Lat/Lon of a StartPoint
lat_s  = 10;   % [deg]
lon_s  = 0;    % [deg]

LAT_s = deg2rad(lat_s);      % [rad] 
LON_s = deg2rad(lon_s);      % [rad] 

LST_s = GST+LON_s;

R_L_I_s = dcm(2,-LAT_s)*dcm(3,LST_s);
r_s    = R_L_I_s'*[1 ;0 ;0];

fprintf('Start: %6.4f | %6.4f | \n',lat_s,lon_s)

% Lat/Lon of a End Point
lat_f  = 55;     % [deg]
lon_f  = 55;    % [deg]

LAT_f = deg2rad(lat_f);      % [rad] 
LON_f = deg2rad(lon_f);      % [rad] 

LST_f = GST+LON_f;

R_L_I_f = dcm(2,-LAT_f)*dcm(3,LST_f);
r_f     = R_L_I_f'*[1 ;0 ;0];

fprintf('Final: %6.4f | %6.4f | \n',lat_f,lon_f)

% Lat/Lon of a mid Point
latlon_m = latlon2mid([lat_s lon_s],[lat_f lon_f]);

lat_m = latlon_m(1);
lon_m = latlon_m(2);

LAT_m = deg2rad(lat_m);
LON_m = deg2rad(lon_m);

LST_m = GST + LON_m;

R_L_I_m = dcm(2,-LAT_m)*dcm(3,LST_m);
r_m     = R_L_I_m'*[1 ;0 ;0];

fprintf('Mid:   %6.4f | %6.4f | \n',lat_m,lon_m)

% Lat/Lon of a int Point
latlon_int = latlon2int([lat_s lon_s],[lat_f lon_f], 0.25);

lat_int = latlon_int(1);
lon_int = latlon_int(2);

LAT_int = deg2rad(lat_int);
LON_int = deg2rad(lon_int);

LST_int = GST + LON_int;

R_L_I_int = dcm(2,-LAT_int)*dcm(3,LST_int);
r_int     = R_L_I_int'*[1;0;0];

fprintf('Int:   %6.4f | %6.4f | \n',lat_int,lon_int)

%% Intermidiate Points by Greater Circle
n = 10;

for i=1:1:n

latlon_arry = latlon2int([lat_s lon_s],[lat_f lon_f], i/n);

lat_arry = latlon_arry(1);
lon_arry = latlon_arry(2);

LAT_arry = deg2rad(lat_arry);
LON_arry = deg2rad(lon_arry);

LST_arry = GST + LON_arry;

R_L_I_arry = dcm(2,-LAT_arry)*dcm(3,LST_arry);
r_arry(:,i)     = R_L_I_arry'*[1;0;0];
end

%%
dLAT = LAT_f-LAT_s;    % [rad]
dLON = LST_f-LST_s;    % [rad]
 
% Azimuth
alp = atan2(tan(dLAT),sin(dLON)); % [rad] dAzimuth
Azm = latlon2azm([lat_s lon_s],[lat_f lon_f]);

dpsi = log(tan(pi/4 + LAT_f/2)/tan(pi/4 + LAT_s/2));
theta = atan2(dLON,dpsi);

fprintf('\nAzm:  %8.8f | %8.8f | %8.8f |\n',alp,Azm, theta)

R_n_i = dcm(1,alp)*dcm(3,LST_s);  % [] Rotation Matrix from Inertial to Circle Plane                    

n_i = [  sin(LST_s)*sin(alp);
        -cos(LST_s)*sin(alp);
            cos(alp)];


% Beta Angle
beta1 = acos(dot(R_L_I_s'*[1;0;0],n_i));
beta2 = acos(cos(alp)*sin(LAT_s));
beta  = beta2;
fprintf('beta:  %8.6f | %8.6f | \n',beta1, beta2)


% Circle Radius between two points
Rc1 = Re*sin(beta);
Rc2 = Re*sqrt(1-(cos(alp))^2*(sin(LAT_s))^2);

Rc  = Rc2;

fprintf('Rc:    %8.8f | %8.8f | \n',Rc1,Rc2)

%  Distance
dist1 = Re * latlon2dist([lat_s lon_s],[lat_f lon_f]); % GREATER CIRCLE THEORY

dpsi = log(tan(pi/4+LAT_f/2)/tan(pi/4+LAT_s/2));
if (dpsi ~= 0)
q    = dLAT/dpsi;
else
q    = cos(dLAT);
end
dist2    = sqrt(dLAT^2 + q^2*dLON^2);                   % RHUMBLINE

dist = dist2;
fprintf('Dist:  %8.6f | %8.6f |\n',dist1,dist2)


% Angle Between Points
tangle = latlon2ang([lat_s lon_s],[lat_f lon_f]);  %[rad]
fprintf('ang:  %6.6f |  \n',tangle)

%% RUSTY
nvector = cross(r_s,r_f); % [ ] 
angle = vangle(r_s,r_f)   % [rad] total angle
t_dwell = 100;            % [s]
t_step = 10;

step = t_dwell/t_step
dangle = angle/step
omega_dwell = angle/t_dwell*vnorm(cross(r_s,r_f))  %[rad/s] CONST
rr(:,1) = r_s
for i=1:1:t_dwell/t_step
 rr(:,i) = vnorm(rr(:,i));
v_perp = cross(omega_dwell,rr(:,i));
rr(:,i+1) = rr(:,i) + v_perp*t_step;
end


%% BST
t_dwell = 100;           % [s]
t_step      = 10;

v_dwell = dist/t_dwell; % [m/s] ground speed
w_dwell = v_dwell/Rc;   % [rad/s] angular velocity
dangle  = w_dwell*t_step;   % [rad] angle step assuming small steps


lat_array = LAT_s;
lon_array = LST_s;

for i=1:1:t_dwell/t_step

    tangle = i*dangle;
    
    lat_array = asin(sin(dangle)*sin(alp)) + lat_array;
    lon_array = atan(tan(dangle)*cos(alp)) + lon_array;
    
    fprintf('|%6.7f|%6.7f|%6.7f|%6.7f|\n',lat_array,lon_array,asin(sin(dangle)*sin(alp)),atan(tan(dangle)*cos(alp)));
    
    R_L_I_array = dcm(2,-lat_array)*dcm(3,lon_array);
    r_array(:,i)    = R_L_I_array'*[1;0;0];
    
end

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


[r_sun_mod,rtasc_sun,decl_sun] = jdut2sun(JD_UTC);

fprintf('Sun Vector (MOD): %.12f %.12f %.12f\n',r_sun_mod);
        
%% ECI
vmod = [0; 0; 0];
amod = [0; 0; 0];
[r_sun_eci,v_sun_eci,a_sun_eci] = mod2eci(r_sun_mod,vmod,amod,T_TT );

fprintf('Sun Vector (ECI): %.12f %.12f %.12f\n',r_sun_eci);

%% SIMULINK
dt   = 1;
tdur = 60*60*CONST.solarday;
sim('earth_model',tdur);

%% SIMULATION
close all 
% Figure Setting
fig = figure;
screensize = get(0,'ScreenSize');
set(fig,'Position',[screensize(3)*0 0 screensize(4)*0.8 screensize(4)*1]);
grid on; axis fill;
cameratoolbar('SetMode','orbit')   
set(gca,'Position',[0 0 1 1]); % Set Position of Graph
set(gca,'CameraViewAngle',3);  % Set Zoom of Graph
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
[S_sun, S_sun_lab] = plotvector(S_I(:,1), [0;0;0], [1 0.75 0], 'Sun_E_C_I',2);
% S_mod = plotvector(r_sun_mod, [0;0;0], [1 0 0], 'Sun_M_O_D',2);

% Earth Frame 
plotvector(R_E_I(:,:,1)'*[1 ;0 ;0], [0 0 0], [1 ;0.75 ;0.75], 'X_s',1.1);
plotvector(R_E_I(:,:,end)'*[1 ;0 ;0], [0 0 0], [1 ;0 ;0], 'X_f',1.5);
[X_ecf,X_ecf_lab] = plotvector(R_E_I(:,:,1)'*[1 ;0 ;0], [0 0 0], 'r', 'X_e_c_f');
[Y_ecf,Y_ecf_lab] = plotvector(R_E_I(:,:,1)'*[0 ;1 ;0], [0 0 0], 'r', 'Y_e_c_f');
[Z_ecf,Z_ecf_lab] = plotvector(R_E_I(:,:,1)'*[0 ;0 ;1], [0 0 0], 'r', 'Z_e_c_f');

%local Points
R_start = plotvector(r_s, [0 0 0], 'b', 'start');
R_end   = plotvector(r_f, [0 0 0], 'b', 'end');
R_mid   = plotvector(r_m, [0 0 0], [0.5 0.4 0.2], 'mid');

plotvector(r_int,[0 0 0],'r','int',2);

% n vector
plotvector(R_n_i'*[0;0;1],[0 0 0],color('orange'),'n',2);
plotvector(n_i,[0 0 0],'r','n_i',2);

plotvector(nvector,[0 0 0],'k','n',1);
plotvector(omega_dwell,[0 0 0],'k','\omega',1.2);

% plot array
for i=1:1:t_dwell/t_step
plotvector(r_array(:,i),[0 0 0],'r','n',1);

end
for i=1:1:t_dwell/t_step
plotvector(r_arry(:,i),[0 0 0],'m','arr',1.1);
end

for i=1:1:t_dwell/t_step
plotvector(rr(:,i),[0 0 0],'b','arr',1.1);
end

for i=1:dt:(length(tout)-1)
% Update ECF Frames
updatevector(X_ecf, R_E_I(:,:,i)'*[1 ;0 ;0], [0 0 0]);
updatevector(Y_ecf, R_E_I(:,:,i)'*[0 ;1 ;0], [0 0 0]);
updatevector(Z_ecf, R_E_I(:,:,i)'*[0 ;0 ;1], [0 0 0]);
set(X_ecf_lab,'Position',R_E_I(:,:,i)'*[1 ;0 ;0]);
set(Y_ecf_lab,'Position',R_E_I(:,:,i)'*[0 ;1 ;0]);
set(Z_ecf_lab,'Position',R_E_I(:,:,i)'*[0 ;0 ;1]);

% Update Sun Frame
updatevector(S_sun, S_I(:,i),  [0 0 0],2);
set(S_sun_lab,'Position',S_I(:,i)*2);
    drawnow;
end
