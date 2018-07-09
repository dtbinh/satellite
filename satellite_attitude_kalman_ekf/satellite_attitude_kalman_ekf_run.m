%% DESIGN SIMULATION NATO-SATELLITE ATTITUDE DETERMINATION SYSTEM
close all
clear all
clc

%% GLOBAL

R2D = 180/pi;
D2R = pi/180;

%% SIMULATION PARAMETERS

Euler_I_B_0  = [-45/180*pi ;0 ;0]; % [rad] Euler Angle from Body Frame to Inertial Frame (Initial)
Euler_REF    = [0 ;0 ;0]; % [rad] Euler Angle Reference

%% CONTROLLER GAIN
Kp  = 0.25;
Kd1 = 0.15;
Kd2 = 0.15;
Kd3 = 0.15;
Ki  = 0.15;

%% TORQUE TOGGLE SWITCH

Tgg_toggle    = 1; % Toggle Torque Gravity Gradient 
Taero_toggle  = 1; % Toggle Torque Aerodynamic
Tsolar_toggle = 1; % Toggle Torque Solar

%% ERROR TOGGLE SWITCH

werrOn  = 1; % Toggle Error Angular Velocity
berrOn  = 1; % Toggle Errot Bias
qerrOn  = 1; % Toggle Error Quaternion

%% CONSTANT PARAMETERS
global CONST;

CONST.mu         = 398.6004118e12;          % [m^3/s^2] Earth's standard gravitational parameter
CONST.mu_moon    = 4.902802953597e12;       % [m^3/s^2] Moon's standard gravitational parameter
CONST.mu_sun     = 1.327122E20;             % [m^3/s^2] Sun's standard gravitational parameter
CONST.Re         = 6.378137E6;              % [m] Earth radius
CONST.Rs         = 1.4959787e11;            % [m] Solar radius
CONST.J2         = 1.08262668355E-3;        % [-] J2 term
CONST.J3         = -2.53265648533E-6;       % [-] J3 term
CONST.J4         = -1.61962159137E-6;       % [-] J4 term
CONST.SolarP     = 4.51e-6;                 % [N/m^2] solar wind pressure
CONST.SOLARSEC   = 806.81112382429;         % TU
CONST.w_earth    = [0;0;0.000072921158553]; % [rad/s] earth rotation 
CONST.Cd         = 2.5;                     % [-] Coefficient of Drag
CONST.Cr         = 0.6;                     % [-] Coefficient of Reflect
CONST.OmegaDot   = 1.991e-7;                % [rad/s] ascending node advance for sun-synch
CONST.gamma      = 23.442/180*pi;           % [rad] Spin Axis of Ecliptic Plane
CONST.dm         = 7.94e22;                 % [Am^2] Earth Dipole Magnetic Moment
CONST.mui        = 4*pi*1e-7;               % [kgm/A^2/s^2] Earth Permeability of Free Space
CONST.Bo         = CONST.mui*CONST.dm/4/pi; % [-] Magnetic Constant for magnetic field calculation
CONST.u_0        = pi/2;                   % [rad] Initial Sun Ascension (pi/2 - Summer, pi - Autumn, 3*pi/2 
CONST.I          = diag([0.2 0.2 0.2]);  % [kgm^2] Spacecraft Moments of Inertia

%% ORBITAL ELEMENTS
h_p = 499.99e3; % [m] Altitude Perigee 
h_a = 500.01e3; % [m] Altitude Apogee

RAAN = 0;  % [rad] Initial Right Ascention - Angle on Equatorial Plane from Vernal Equinox to Ascending Node
w    = 0;  % [rad] Initial Argument of perigee - Angle on Orbital Plane from Ascending Node to Perigee
TAo  = 0;  % [deg] Initial True Anomaly - Angle on Orbital Plane from Perigee to Satellite

Rp  = CONST.Re+h_p;              % [m] radius of perigee
Ra  = CONST.Re+h_a;              % [m] radius of apogee
e   = (Ra-Rp)/(Ra+Rp);           % [m/m] eccentricity
a   = (Ra+Rp)/2;                 % [m] semi-major axis
ho  = sqrt(a*CONST.mu*(1-e^2));  % [mˆ2/s] Initial Angular momentum
P   = 2*pi*sqrt(a^3/CONST.mu);   % [sec] Orbit Period
w_O = 2*pi/P;                   % [rad/s] Orbit Angular Velocity

CONST.P   = P;                      % [sec] Orbit Period
CONST.a   = a;                      % [m] semi-major axis
CONST.w_O = w_O;                    % [rad/s] Orbit Angular Velocity

CONST.RAAN = RAAN;                  % [rad] Initial Right Ascention - Angle on Equatorial Plane from Vernal Equinox to Ascending Node

i = acos((CONST.OmegaDot*(1-e^2)^2*a^(7/2))/(-3/2*sqrt(CONST.mu)*CONST.J2*CONST.Re^2)); % [rad] Orbit Inclination required for Sun-synchornous (eqn 4.47 from Curtis)
i = 45/180*pi;
[R_0,V_0,Q] = sv_from_coe(CONST.mu,[ho e RAAN i w TAo]);% initial orbital state vector in ECF Frame- computes the magnitude of state vector (r,v) from the classical orbital elements (coe) 

%% INITIAL CONDITIONS
R_O_I   = DCM(1,-90*D2R)*DCM(3,(TAo+90)*D2R)*DCM(1,i)*DCM(3,RAAN); % [3x3] Rotation Matrix from Inertial (X axis is vernal equinox) to Orbit Frame (x is orbit direction)
Euler_O_I_0 = R2eul(R_O_I,'ZYX');   % [rad] Initial Euler Angle transforming from Inertial Frame to Orbit Frame (ZYX means rotate X, then Y, then Z)
R_I_B_0 = eul2R(Euler_I_B_0,'ZYX'); % Transformation Matrix from Body to Inertial (Initial)   
R_O_I_0 = eul2R(Euler_O_I_0,'ZYX'); % Transformation Matrix from Inertial to Body (Initial)
R_O_B_0 = R_O_I_0*R_I_B_0;          % Transformation Matrix from Body to Orbital (Initial)

q_I_B_0 = R2q(R_I_B_0,'tsf','wxyz'); % Quaternion Rotation of Body Frame from Inertial Frame
q_O_I_0 = R2q(R_O_I_0,'tsf','wxyz'); % Quaternion Rotation of Inertial Frame From Body Frame
q_O_B_0 = R2q(R_O_B_0,'tsf','wxyz'); % Quaternion Rotation of Body Frame from Orbit Frame

q_B_I_0 = [q_I_B_0(1);-q_I_B_0(2);-q_I_B_0(3);-q_I_B_0(4)];
q_B_O_0 = [q_O_B_0(1);-q_O_B_0(2);-q_O_B_0(3);-q_O_B_0(4)];

Euler_O_B_0 = R2eul(R_O_B_0,'ZYX'); % Euler Angles from Body to Orbital (Initial)

R_E_M   = DCM(1,-11.5/180*pi); % Rotation Matrix from Magnetic to Earth (Constant)

% Orbital Frame Angular Rate
w_O_OI_0 = [0;-w_O;0];             % [rad] Orbital Frame Angular Rate relative to Inertial Frame in Orbital Frame
w_B_OI_0 = R_O_B_0'*w_O_OI_0; % [rad] Orbital Frame Angular Rates relative to Inertial Frame in Body Frame

% Body Frame Angular Rate
w_B_BO_0 = [0.00;0.00;0.00];               % [rad] Body Frame Angular Rates relative to Orbital Frame in Body Frame 
w_B_BI_0 = w_B_BO_0 + w_B_OI_0;   % [rad] Body Frame Angular Rate relative to Inertial Frame in Body Frame

%% SENSORS FLAG
mflag(1) = 0; % Star Tracker
mflag(2) = 1; % Sun Sensor 1
mflag(3) = 1; % Sun Sensor 2
mflag(4) = 1; % Sun Sensor 3
mflag(5) = 1; % Sun Sensor 4
mflag(6) = 1; % Magnetometer

CONST.mflag = mflag; % Set as CONST global variable
%% GYRO SENSOR
GYRO_Bias = (3*randn(3,1))*pi/180;  % [rad/s] +3 deg
N_ARW     = (0.029)*pi/180;         % [rad/s^0.5] Standard Deviation
K_RRW     = (0.0002)*pi/180;        % [rad/s^1.5] Standard Deviation
ARW       = N_ARW^2;                % [rad^2/s] Angular White Noise Variance
RRW       = K_RRW^2/3;              % [rad^2/s^3] Bias Variance
Gg        = eye(3).*(-0.01+0.02*rand(3))+ ...
            (ones(3,3)-eye(3)).*(-0.0006+0.0012*rand(3)); % [%] 3x3 Matrix diagonal values of G are the percent error in scale factor and the off-diagonal values of G are the percent error of misalignment
gyro_max = 75/180*pi;

%% STAR TRACKER
sigST = 70/3/60/60*pi/180; % [rad]   arcsec to rad (3 sigma)
varST = sigST^2;           % [rad^2] Covariance of StarTracker
Gst   = eye(3).*(-0.02+0.04*rand(3)) +...
         (ones(3,3)-eye(3)).*(-0.0028+0.0056*rand(3)); % percent
dt_st = 	1;                 % [s] Sampling Time of Star Tracker

%% SUN SENSOR
% Gss  = eye(2).*(-0.02+0.04*rand(2)) +...
%         (ones(2,2)-eye(2)).*(-0.0028+0.0056*rand(2)); % percent
SSaxis   = [1; 1; 1; 1];             % [axis] Sun Sensor rotation axis 1 = x, 2 = y, 3 = z
SSangles = [-30;-150; 30; 150]*pi/180; % [rad] Sun Sensors fram angles              
R_S1_B = DCM(SSaxis(1),SSangles(1));   % [-] Rotation Matrix from Body Frame to Sun Sensor 1 Frame (Z-axis of sun sensor aligned with X-axis)
R_S2_B = DCM(SSaxis(2),SSangles(2));   % [-] Rotation Matrix from Body Frame to Sun Sensor 2 Frame (Z-axis of sun sensor aligned with -X-axis)
R_S3_B = DCM(SSaxis(3),SSangles(3));   % [-] Rotation Matrix from Body Frame to Sun Sensor 3 Frame (Z-axis of sun sensor aligned with -X-axis)
R_S4_B = DCM(SSaxis(4),SSangles(4));   % [-] Rotation Matrix from Body Frame to Sun Sensor 4 Frame (Z-axis of sun sensor aligned with -X-axis)


FOV   = 70; % [deg] Field of View
sigSS = 0.1; % [deg] Standard Deviation
% J     = Bessel(sigSS/3,FOV).*pi/180;
dt_ss = 1/5;

CONST.SSaxis = SSaxis;        % [axis] Sun Sensor rotation axis 1 = x, 2 = y, 3 = z
CONST.SSangles = SSangles;    % [deg] Sun Sensors fram angles
%% MAGNETOMETER
sigMag   = 1.25e-7;                                       % [T] Standard Deviation
Mag_bias = (4*randn(3,1))*1e-7;                           % [T], +/- 4 mguass
Gmg      = eye(3).*(-0.02+0.04*rand(3)) +...
            (ones(3,3)-eye(3)).*(-0.0028+0.0056*rand(3)); % [%] MisAlignment
magn_max = 100/180*pi; 
dt_mg    = 1/20;                                          

%% KALMAN FILTER
dt     = 1/20;               % [sec] (20 Hz) Model speed
dt_ekf  = dt;                % [sec] (20 Hz) EKF speed
CONST.dt     = dt;
CONST.dt_ekf = dt_ekf;
CONST.sig_v  = sqrt(ARW);    % [rad/sec^(1/2)] ARW gyro
CONST.sig_u  = sqrt(RRW);    % [rad/sec^(3/2)] RRW gyro
CONST.sig_st = sigST;        % [rad] Star Tracker Error
CONST.sig_ss = sigSS*pi/180; % [rad] Sun Sensor Error
CONST.sig_mg = sigMag;       % [tesla] magnetometer error

ReferenceOmega = w_B_OI_0;


%% SOLVER
fprintf("\nsatellite_attitude_kalman_ekf_model running\n");
tdur = P;              
sim('satellite_attitude_kalman_ekf_model',tdur);

%% POST PROCESSING
close all
R = R/CONST.Re;  % Position Vector of spacecraft

for i=1:1:length(tout)
Rx(i) = R(1,1,i); % Position Vector X of spacecraft
Ry(i) = R(2,1,i); % Position Vector Y of spacecraft
Rz(i) = R(3,1,i); % Position Vector Z of spacecraft

vr(i) = norm(Vr(:,1,i));
vt(i) = norm(Vt(:,1,i));
v(i)  = sqrt(vr(i)^2+vt(i));

q_B_I_error(:,i)  = q_B_I_f(:,i) - q_B_I(:,i);
e_B_I_error(:,i)  = e_B_I_f(:,i) - e_B_I(:,i);
w_B_BI_error(:,i) = w_B_BI_f(:,i) - w_B_BI(:,i);
bias_error(:,i)   = bias_f(:,i) - bias(:,i);
end
%% PLOT
satellite_attitude_kalman_ekf_plot
%% SIMULATION
% Figure Setting
fig = figure;
screensize = get(0,'ScreenSize');
set(fig,'Position',[0 0 screensize(4)*0.8 screensize(4)*0.8]);
grid on; axis fill;
cameratoolbar('SetMode','orbit')   
set(gca,'Position',[0 0 1 1]); % Set Position of Graph
set(gca,'CameraViewAngle',6); % Set Zoom of Graph
axis(3.5*[-1 1 -1 1 -1 1]);    % Set Limit of Axis  

% Earth Centered Inertial Frame
X_eci = plotVector([1 ;0 ;0], [0 0 0], 'k', 'X_eci');
Y_eci = plotVector([0 ;1 ;0], [0 0 0], 'k', 'Y_eci');
Z_eci = plotVector([0 ;0 ;1], [0 0 0], 'k', 'Z_eci');


% Earth Centered Fixed Frame
X_ecf = plotVector(R_I_E(:,:,1)*[1 ;0 ;0], [0 0 0], 'r');
Y_ecf = plotVector(R_I_E(:,:,1)*[0 ;1 ;0], [0 0 0], 'r');
Z_ecf = plotVector(R_I_E(:,:,1)*[0 ;0 ;1], [0 0 0], 'r');

% Earth Centered Magnetic Frame
X_emf = plotVector(R_I_E(:,:,1)*R_E_M*[1 ;0 ;0], [0 0 0], 'm');
Y_emf = plotVector(R_I_E(:,:,1)*R_E_M*[0 ;1 ;0], [0 0 0], 'm');
Z_emf = plotVector(R_I_E(:,:,1)*R_E_M*[0 ;0 ;1], [0 0 0], 'm');

% Satellite Body Frame
[X_sat,X_sat_lab] = plotVector(R_B_I(:,:,1)'*[1 ;0 ;0], R(:,1,1), 'b', 'x_b',0.5);
[Y_sat,Y_sat_lab] = plotVector(R_B_I(:,:,1)'*[0 ;1 ;0], R(:,1,1), 'b', 'y_b',0.5);
[Z_sat,Z_sat_lab] = plotVector(R_B_I(:,:,1)'*[0 ;0 ;1], R(:,1,1), 'b', 'z_b',0.5);

% Orbital Frame
[X_orb,X_orb_lab] = plotVector(R_O_I(:,:,1)'*[1 ;0 ;0], R(:,1,1), 'g', 'x_o',0.5);
[Y_orb,Y_orb_lab] = plotVector(R_O_I(:,:,1)'*[0 ;1 ;0], R(:,1,1), 'g', 'y_o',0.5);
[Z_orb,Z_orb_lab] = plotVector(R_O_I(:,:,1)'*[0 ;0 ;1], R(:,1,1), 'g', 'z_o',0.5);

% Magnetic Field 
[B_mag,B_mag_lab] = plotVector(B_I(:,1), R(:,1,1), 'm', 'B',0.25);

% Earth Centered Sun Frame
[S_sun, S_sun_lab] = plotVector(S_I(:,1), [0;0;0], 'y', 'Sun',2);

% Earth Centered Sun Frame
[S_hat, S_hat_lab] = plotVector(-S_I_hat(:,1), [0;0;0], color('orange'), 'S_h_a_t',2);

% Eclipse Status
eclipse_lab = text(1.5,1.5,1.5,strcat('Eclipse:',int2str(ECLIPSE(1))));


% Velocity Vector
% Vtplot = plotVector(Vt(:,1,1), R(:,1,1), 'r'); % Tangential Vector
% Vrplot = plotVector(Vr(:,1,1), R(:,1,1), 'r'); % Radial Vector
Vplot  = plotVector(V(:,1,1), R(:,1,1), 'r');  % Velocity Vector


% Satellite Position
R_sat = plotPosition(R,'b','s');
hold on;

% Plot Satellite Orbit Track
plot3(Rx,Ry,Rz,'-.')

% Create Earth Sphere
radius  = 1;
[x,y,z] = sphere(20); 
earth   = surf(radius*x,radius*y,radius*z);
set(earth,'facecolor','none','edgecolor',0.7*[1 1 1],'LineStyle',':');  

% MAGNETIC FIELD PLOT
%  MagneticField();

% UPDATE SIMULATION PLOT
d=10;
for i=1:10*d:(length(tout)-1)

rotate(earth,[0 0 1],0.005*d)
% Update Axes Label


% Update Satellite Position
updatePosition(R_sat, R(:,1,i));

% Update ECF Frames
updateVector(X_ecf, R_I_E(:,:,i)*[1 ;0 ;0], [0 0 0]);
updateVector(Y_ecf, R_I_E(:,:,i)*[0 ;1 ;0], [0 0 0]);
updateVector(Z_ecf, R_I_E(:,:,i)*[0 ;0 ;1], [0 0 0]);

% Update EMF Frames
updateVector(X_emf, R_I_E(:,:,i)*R_E_M*[1 ;0 ;0], [0 0 0]);
updateVector(Y_emf, R_I_E(:,:,i)*R_E_M*[0 ;1 ;0], [0 0 0]);
updateVector(Z_emf, R_I_E(:,:,i)*R_E_M*[0 ;0 ;1], [0 0 0]);

% Update Earth Centered Sun Frame
updateVector(S_sun, S_I(:,i),  [0 0 0],2);

% Update Earth Centered Sun Frame
updateVector(S_hat, -S_I_hat(:,i),  [0 0 0],2);

% Update Orbit Frame
updateVector(X_orb, R_O_I(:,:,i)'*[1 ;0 ;0],  R(:,1,i),0.5);
updateVector(Y_orb, R_O_I(:,:,i)'*[0 ;1 ;0],  R(:,1,i),0.5);
updateVector(Z_orb, R_O_I(:,:,i)'*[0 ;0 ;1],  R(:,1,i),0.5);
set(X_orb_lab,'Position',R_O_I(:,:,i)'*[1 ;0 ;0]+R(:,1,i));
set(Y_orb_lab,'Position',R_O_I(:,:,i)'*[0 ;1 ;0]+R(:,1,i));
set(Z_orb_lab,'Position',R_O_I(:,:,i)'*[0 ;0 ;1]+R(:,1,i));

% Update Satellite Body Frame
updateVector(X_sat, R_B_I(:,:,i)'*[1 ;0 ;0],  R(:,1,i),0.5);
updateVector(Y_sat, R_B_I(:,:,i)'*[0 ;1 ;0],  R(:,1,i),0.5);
updateVector(Z_sat, R_B_I(:,:,i)'*[0 ;0 ;1],  R(:,1,i),0.5);
set(X_sat_lab,'Position',R_B_I(:,:,i)'*[1 ;0 ;0]+R(:,1,i));
set(Y_sat_lab,'Position',R_B_I(:,:,i)'*[0 ;1 ;0]+R(:,1,i));
set(Z_sat_lab,'Position',R_B_I(:,:,i)'*[0 ;0 ;1]+R(:,1,i));

% Update Satellite Body Frame
updateVector(B_mag, B_I(:,i),  R(:,1,i));

% Update Eclipse
set(eclipse_lab,'String',strcat('Eclipse:',int2str(ECLIPSE(i))));

% Update Velocity Vectors
% updateVector(Vtplot, Vt(:,1,i), R(:,1,i));
% updateVector(Vrplot, Vr(:,1,i), R(:,1,i));
updateVector(Vplot, V(:,1,i), R(:,1,i));

drawnow;  
    
end

