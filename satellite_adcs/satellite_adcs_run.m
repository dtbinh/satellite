%% DESIGN SIMULATION NANO-SATELLITE ATTITUDE DETERMINATION SYSTEM
clear all
clc
format long
global CONST;

%% MODEL PARAMETER
dt  = 0.5;              % [sec] Model speed (0.5 for normal, 30 for orbit)

%% TORQUE TOGGLE SWITCH

Tgg_toggle      = 1;    % Toggle Torque Gravity Gradient 
Taero_toggle    = 1;    % Toggle Torque Aerodynamic
Tsolar_toggle   = 1;    % Toggle Torque Solar
Tnoise_toggle   = 1;    % Toggle Torque Noise
sigTorq         = 5e-8;

Tcontrol_toggle = 1;    % Toggle Torque Control

%% TIME
% UTC Time - Universal Time
UTC_sim   = datetime(2000,6,21,6,0,0);                                     % [time] UTC Time
UTC_start = datetime(2000,3,21,6,0,0);                                     % [time] UTC Time
UTC_end   = datetime(2000,3,21,7,0,0);                                     % [time] UTC Time

tgt_dur = datenum(UTC_end - UTC_start)*24*60*60;                           

JD_UTC  = jd(UTC_sim.Year,UTC_sim.Month,UTC_sim.Day,UTC_sim.Hour,UTC_sim.Minute,UTC_sim.Second);   % [day]
T_UTC   = (JD_UTC -2451545.0 )/36525;                                      % [century]
d_AT    = 33.0;                                                            % [sec] From Astronomical Almanac 2006:K9
d_UT1   = 0.2653628;                                                       % [sec]
[UT1, T_UT1, JD_UT1, UTC_sim, TAI, TT, T_TT, JD_TT, T_TDB, JD_TDB] ...
    = convtime(UTC_sim.Year,UTC_sim.Month,UTC_sim.Day,UTC_sim.Hour,UTC_sim.Minute,UTC_sim.Second,d_UT1, d_AT);

CONST.JD_UT1 = JD_UT1;
CONST.JD_UTC = JD_UTC;
CONST.T_TT   = T_TT;

CONST.sidereal = 23.934469583333332; % [hrs] Hours in a Sidereal Day
CONST.solarday = 24.00000000;        % [hrs] Hours in a Solar Day 

[r_sun_mod,rtasc_sun,decl_sun]  = jdut2sun(JD_UTC);
[r_sun_eci,v_sun_eci,a_sun_eci] = mod2eci(r_sun_mod,[0; 0; 0],[0; 0; 0],T_TT );

%% CONSTANT PARAMETERS

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
CONST.u_0        = pi/2;%rand(1,1)*pi;      % [rad] Initial Sun Ascension (pi/2 - Summer, pi - Autumn, 3*pi/2 

%% SATELLITE MOMENTS OF INERTIA
m  = 2.00;  % [kg] Satellite Mass
dx = 0.20;  % [m] Length X
dy = 0.40;  % [m] Length Y
dz = 0.40;  % [m] Length Z

Ix = (m/12)*(dy^2+dz^2); % [kg.m^2] X-axis Inertia
Iy = (m/12)*(dx^2+dz^2); % [kg.m^2] Y-axis Inertia
Iz = (m/12)*(dx^2+dy^2); % [kg.m^2] Y-axis Inertia
  
I  = diag([Ix Iy Iz]);   % [kg.m^2] Inertia 

CONST.I  = I;     % [kgm^2] Spacecraft Moments of Inertia
CONST.dx = dx;    % [m] Length X
CONST.dy = dy;    % [m] Length Y
CONST.dz = dz;    % [m] Length Z

%% COIL PARAMETERS AND CONTROL ALLOCATION MATRIX
V_max = 5;                % [V] Max voltage

Ncoil = [355;355;800];    % [-] Number of turns in Coil
Acoil = [0.08*0.18;
         0.08*0.18;
         0.08*0.08];      % [m^2] Area of Coil 
Rcoil = [110;110;110];    % [Ohm] Resistance in Coil 
i_max = [V_max/Rcoil(1);
         V_max/Rcoil(2);
         V_max/Rcoil(3)]; 

% K Coil Matrix
K_coil = [Ncoil(1)*Acoil(1)/Rcoil(1)   0        0        ;     
             0       Ncoil(2)*Acoil(2)/Rcoil(2) 0        ;    
             0       0        Ncoil(3)*Acoil(3)/Rcoil(3) ];    

CONST.Ncoil = Ncoil;    % [-]   Number of Coils
CONST.Acoil = Acoil;    % [m^2] Area of Coil
CONST.V_max = V_max;    % [V]   Voltage Maximum
CONST.K     = K_coil;   % [3x3] K coil Matrix
CONST.i_max = i_max;    % [A]   Ampere Maximum



%% ORBITAL ELEMENTS
h_p = 499.9999999e3; % [m] Altitude Perigee 
h_a = 500.0000001e3; % [m] Altitude Apogee

RAAN = 0;  % [rad] Initial Right Ascention - Angle on Equatorial Plane from Vernal Equinox to Ascending Node
w    = 0;  % [rad] Initial Argument of perigee - Angle on Orbital Plane from Ascending Node to Perigee
TAo  = 0;  % [deg] Initial True Anomaly - Angle on Orbital Plane from Perigee to Satellite

Rp  = CONST.Re + h_p;            % [m] radius of perigee
Ra  = CONST.Re + h_a;            % [m] radius of apogee
ecc = (Ra-Rp)/(Ra+Rp);           % [m/m] eccentricity
a   = (Ra+Rp)/2;                 % [m] semi-major axis

P   = 2*pi*sqrt(a^3/CONST.mu);   % [sec] Orbit Period
w_O = 2*pi/P;                    % [rad/s] Orbit Angular Velocity

CONST.P   = P;                   % [sec] Orbit Period
CONST.a   = a;                   % [m] semi-major axis
CONST.w_O = w_O;                 % [rad/s] Orbit Angular Velocity
CONST.RAAN = RAAN;               % [rad] Initial Right Ascention - Angle on Equatorial Plane from Vernal Equinox to Ascending Node

% Inclination
incl = acos((CONST.OmegaDot*(1-ecc^2)^2*a^(7/2))/(-3/2*sqrt(CONST.mu)*CONST.J2*CONST.Re^2)); % [rad] Orbit iination required for Sun-synchornous (eqn 4.47 from Curtis)                                                                          % [rad] Orbit iination 
incl = 50/180*pi;

% Initial orbital state vector in ECF Frame- computes the magnitude of state vector (r,v) from the classical orbital elements (coe)
[R_0,V_0] = coe2rv(a, ecc, incl,RAAN, w, TAo,'curtis'); 

CONST.incl = incl;
CONST.ecc  = ecc;

%% INITIAL CONDITIONS
R_O_I   = dcm(1,-90*deg2rad)*dcm(3,(TAo+90)*deg2rad)*dcm(1,incl)*dcm(3,RAAN); % [3x3] Rotation Matrix from Inertial (X axis is vernal equinox) to Orbit Frame (x is orbit direction)

Euler_I_B_0  = [ 0 ; deg2rad(-23.5) ; pi/2 ];   % [rad] Euler Angle from Body Frame to Inertial Frame (Initial)
Euler_O_I_0 = dcm2eul(R_O_I,'zyx');             % [rad] Initial Euler Angle transforming from Inertial Frame to Orbit Frame (ZYX means rotate X, then Y, then Z)

R_I_B_0 = eul2dcm(Euler_I_B_0,'zyx');  % Transformation Matrix from Body to Inertial (Initial)   
R_O_I_0 = eul2dcm(Euler_O_I_0,'zyx');  % Transformation Matrix from Inertial to Body (Initial)
R_O_B_0 = R_O_I_0*R_I_B_0;             % Transformation Matrix from Body to Orbital (Initial)

q_I_B_0 = dcm2q(R_I_B_0,'tsf','xyzw'); % Quaternion Rotation of Body Frame from Inertial Frame
q_O_I_0 = dcm2q(R_O_I_0,'tsf','xyzw'); % Quaternion Rotation of Inertial Frame From Body Frame
q_O_B_0 = dcm2q(R_O_B_0,'tsf','xyzw'); % Quaternion Rotation of Body Frame from Orbit Frame

q_B_I_0 = qinvert(q_I_B_0,'xyzw');
q_B_O_0 = qinvert(q_O_B_0,'xyzw');

Euler_O_B_0 = dcm2eul(R_O_B_0,'zyx'); % Euler Angles from Body to Orbital (Initial)

R_E_M   = dcm(1,-11.5/180*pi); % Rotation Matrix from Magnetic to Earth (Constant)

% Orbital Frame Angular Rate
w_O_OI_0 = [0;-w_O;0];         % [rad] Orbital Frame Angular Rate relative to Inertial Frame in Orbital Frame
w_B_OI_0 = R_O_B_0'*w_O_OI_0;  % [rad] Orbital Frame Angular Rates relative to Inertial Frame in Body Frame

% Body Frame Angular Rate
w_B_BO_0 = dps2rps(0.5)*[randn(1,1);randn(1,1);randn(1,1)]; % [rad/s] Body Frame Angular Rates relative to Orbital Frame in Body Frame 
w_B_BO_0 = dps2rps(0.5)*[1;1;1];
w_B_BI_0 = w_B_BO_0 + w_B_OI_0;   % [rad] Body Frame Angular Rate relative to Inertial Frame in Body Frame



%% GYRO SENSOR
GYRO_Bias   = [1.5;1.2;-3.3]*deg2rad;       % [rad/s] Actual Offset at Reference Temperature
GYRO_max    = 75/180*pi;                   % [rad/s] Maximum Gyro Rate 
temp_coeff  = [0.01; 0.02; 0.005]*pi/180;  % [rad/s/C] Offset Temperature Coefficient
temp_ref    = 20;                          % [C] Temperature Reference
dt_gyro     = 0.1;                         % [s] Sampling Time Interval of Gyro
N_ARW       = (0.0029)*pi/180;             % [rad/s^0.5]    Parameter N variance coefficient at tau = 1 along +1/2
K_RRW       = (0.0002)*pi/180;             % [rad/s^1.5]    Parameter K variance coefficient at tau = 3 along -1/2 slope

N_W         = (1e-3)*pi/180;               % [rad/C/s^0.5]  Standard Deviation Measured
Var_ARW     = N_ARW^2;                     % [rad^2/s]      Variance of Angular White Noise 
Var_RRW     = K_RRW^2/3;                   % [rad^2/s^3]    Variance of Bias Rate
VarW        = N_W^2;                       % [rad^2/s] 

R_G_B   = dcm(2,pi/2);                     % [-] Rotation Matrix from Body Frame to Gyro Frame (Used in Temperature Sensor)

GYRO_Mis = [0.0;0.0;0.0]*pi/180;           % [rad] Gyro Misalignment Euler Angles (3-2-1)
Ug  = dcm(3,GYRO_Mis(3))*dcm(2,GYRO_Mis(2))*dcm(1,GYRO_Mis(1));

update_gyro = 2;
width_gyro  = 2*dt/update_gyro*100;


%% STAR TRACKER
% Star Tracker 1
sig_st(1)   = 30/3/60/60*pi/180;        % [rad]  arcsec to rad (3 sigma)
varST_x(1) = (200/3/60/60*pi/180)^2;   % [rad^2] Covariance of StarTracker (roll)
varST_y(1) = (30/3/60/60*pi/180)^2;    % [rad^2] Covariance of StarTracker (pitch)
varST_z(1) = (30/3/60/60*pi/180)^2;    % [rad^2] Covariance of StarTracker (yaw)
ST1_Mis    = [0.0;0.0;0.0]*pi/180;     % [rad] Star Tracker Misalignment Euler Angles (3-2-1)
Ust1       = eul2dcm(ST1_Mis,'zyx');   % [-] Misalignment Matrix

% Star Tracker 2
sig_st(2)   = 80/3/60/60*pi/180;        % [rad]  arcsec to rad (3 sigma)
varST_x(2) = (150/3/60/60*pi/180)^2;   % [rad^2] Covariance of StarTracker (roll)
varST_y(2) = (15/3/60/60*pi/180)^2;    % [rad^2] Covariance of StarTracker (pitch)
varST_z(2) = (15/3/60/60*pi/180)^2;    % [rad^2] Covariance of StarTracker (yaw)
ST2_Mis    = [0.0;0.0;0.0]*pi/180;     % [rad] Star Tracker Misalignment Euler Angles (3-2-1)
Ust2       = eul2dcm(ST2_Mis,'zyx');   % [-] Misalignment Matrix

dt_st      = dt;                       % [s] Sampling Time of Star Tracker
update_st1 = 30;                        % [s] Update Interval of Star Tracker 1
update_st2 = 30;                        % [s] Update Interval of Star Tracker 2
width_st1  = 5/update_st1*100;
width_st2  = 5/update_st2*100;

if dt_st < dt
    dt_st = dt;
end

%% SB
sig_sb = 30/60/60*pi/180;

%% SUN SENSOR
SSaxis   = [1; 1; 1; 1; 2 ;2];             % [axis] Sun Sensor rotation axis 1 = x, 2 = y, 3 = z
SSangles = [90;-90; 0; 180; -90; 90]*pi/180;  % [rad] Sun Sensors fram angles 
SSnum    = length(SSaxis);           % [ ] Number of Sun Sensors

R_S_B    = zeros(3,3,SSnum);
for i = 1:1:SSnum
R_S_B(:,:,i)   = dcm(SSaxis(i),SSangles(i)); % [-] Rotation Matrix from Body Frame to Sun Sensor Frame 
end

sig_ss = 0.01*pi/180; % [rad] Standard Deviation
dt_ss = dt;           % [sec] Sampling Time of Sun Sensors

CONST.FOV_ss   = 180*pi/180;   % [rad] Field of View of Sun Sensors
CONST.R_S_B    = R_S_B;        % [-] Rotation Matrix from Body Frame to Sun Sensor 1 Frame (Z-axis of sun sensor aligned with X-axis)
CONST.SSaxis   = SSaxis;       % [axis] Sun Sensor rotation axis 1 = x, 2 = y, 3 = z
CONST.SSangles = SSangles;     % [deg]  Sun Sensors fram angles
CONST.SSnum    = SSnum;        % [ ] Number of Sun Sensors

%% MAGNETOMETER
sig_mg   = 1.25e-7;                        % [T] Standard Deviation
Mag_bias = (4*randn(3,1))*1e-7;            % [T], +/- 4 mguass
Gmg      = eye(3).*(-0.02+0.04*rand(3)) +...
            (ones(3,3)-eye(3)).*(-0.0028+0.0056*rand(3)); % [%] MisAlignment
Umg      = eye(3) + Gmg;

dt_mg    = dt;   % [sec] Sampling Time of Magnetometer                                       

CONST.sig_mg = sig_mg;
CONST.Umg    = Umg;

%% TEMPERATURE SENSOR
sigTemp = 0.002;        % [C] Sigma of Temperature Sensor
varTemp = sigTemp^2;    % [C^2] Variance of Temperature Sensor
dt_ts   = dt;           % [sec] Sampling Time of Temperature Sensor

CONST.dt     = dt;               % [sec] (20 Hz) Model speed
CONST.dt_ekf = dt;               % [sec] (20 Hz) EKF speed
CONST.dt_ukf = dt;               % [sec] (20 Hz) EKF speed
CONST.dt_ss = dt_ss;
CONST.sig_v  = sqrt(Var_ARW);    % [rad/s^(1/2)] sigma of process noise - attitude state
CONST.sig_u  = sqrt(Var_RRW);    % [rad/s^(3/2)] sigma of process noise - bias state
CONST.sig_w  = sqrt(VarW);       % [rad/C/s^(1/2)] Standard Deviation Measured
CONST.sig_st = sig_st;            % [rad] Star Tracker 
CONST.sig_ss = sig_ss;            % [rad] Sun Sensor 
CONST.sig_sb = sig_sb;            % [rad] SB 

CONST.varST_x = varST_x;
CONST.varST_y = varST_y;
CONST.varST_z = varST_z;
CONST.sig_w

%% KALMAN FILTER
global FLTR

FLTR.mflag(1) = 1; % Star Tracker 1
FLTR.mflag(2) = 1; % Star Tracker 2
FLTR.mflag(3) = 1; % SB
FLTR.mflag(4) = 0; % Sun Sensor 
FLTR.mflag(5) = 0; % Magnetometer
FLTR.mflag(6) = 1; % Gyroscope


FLTR.qk   = [0;0;0;1];
FLTR.bk   = [0;0;0];
FLTR.wk   = [0;0;0];


% EKF Initial Error Covariance
FLTR.dbekf = 0;
FLTR.ekf = 3;
sig_n = sqrt(sig_st(1)^2  +  sig_ss^2  + sig_mg^2);
FLTR.Pk_ekf = dt^(1/4)*sig_n^(1/2)*(CONST.sig_v^2  +  2*CONST.sig_u*CONST.sig_v*dt^(1/2))^(1/4)*eye(12); 

            
% UKF Initial Error Covariance
FLTR.dbukf = 0;
FLTR.lamda  = 1;
FLTR.alpha  = 1;
FLTR.Pk_ukf = [(1)^2*eye(3)      zeros(3); 
                zeros(3)  (3*pi/180)^2*eye(3)];

% SKF
FLTR.dbskf = 0;
FLTR.Pk_skf   = dt^(1/4)*sig_n^(1/2)*(CONST.sig_v^2  +  2*CONST.sig_u*CONST.sig_v*dt^(1/2))^(1/4)*eye(12);


%% TARGET
CONST.lat = 25;                 % [deg]
CONST.lon = 95;                 % [deg]

GST = jdut2gst(CONST.JD_UT1);    % [rad]    
R_E_I = dcm(3,GST);
R_I_E = R_E_I';
r_tgt = latlon2vec(CONST.lat,CONST.lon,rad2deg(GST)); 

[R_i_r,r1,r2,r3] = triad2dcm(r_tgt-R_0/CONST.Re, V_0);

R_r_i = R_i_r';

CONST.los_vec = [0;0;1];  % LOS of payload in body frame
CONST.vel_vec = [1;0;0];  % velocity vector in orbital frame


%% CONTROLLER
global CTRL_BDOT
global CTRL_EDSP
global CTRL_VDOT
global CTRL_SM
global CTRL_SP
global CTRL_RF
global CTRL_TA
global CTRL_SC

% Bdot Controller
CTRL_BDOT.K_d   = 4e-05;      % [-] Differential Controller Gain 8*w_O^2*(I(2,2)-I(3,3)) =  4.899878019542987e-08

% Energy Dissipitative Controller
CTRL_EDSP.K_d   = 4e-05;      % [-] Differential Controller Gain 8*w_O^2*(I(2,2)-I(3,3)) =  4.899878019542987e-08

% Vdot Controller
CTRL_VDOT.K_d   = 1e-03;

% Reference Controller
CTRL_RF.K_p   = 5e-08;      % [-] Proportional Controller Gain
CTRL_RF.K_d   = 4e-05;      % [-] Differential Controller Gain 8*w_O^2*(I(2,2)-I(3,3))

% Sliding Mode Controller
CTRL_SM.k   = 1.00;         % derivative gain on dqd and part-propotional gain on dq
CTRL_SM.eps = 0.10;         % eps if value is too small, it works like
CTRL_SM.G   = 0.1*eye(3);   % bst_model derivative gain on dw and part-propotional gain on dq
CTRL_SM.g   = 1.0*eye(3);   % bst derivative gain on dw and part-propotional gain on dq
CTRL_SM.type = 'bst_mod';


% Sun Pointing Controller
CTRL_SP.K_p = 1e-03;                       % [ ] Proportional Gain
CTRL_SP.K_v = 1e-02;                       % [ ] Damping Gain
CTRL_SP.S_B_tgt = vnorm([0;0;1]);          % [-] Desired sun vector in Body Frame, should be optimal sun vector
CTRL_SP.w_B_tgt = 0.05*CTRL_SP.S_B_tgt;    % [-] Desired Angular Velocity
CTRL_SP.type    = 'noangmom';

% Three Axis controller
CTRL_TA.k1 = 5e-05;    % [ ] - eigen axis k1 gain              
CTRL_TA.k2 = 5e-05;    % [ ] - eigen axis k2 gain  
CTRL_TA.d1 = 1e-03;    % [ ] - eigen axis d1 gain  
CTRL_TA.d2 = 1e-03;    % [ ] - eigen axis d2 gain 

CTRL_TA.kp = 4e-02;    % [ ] - q feedback proportional gain             
CTRL_TA.kd = 4e-02;    % [ ] - q feedback damping gain

CTRL_TA.type = 'qfeedback';

% Spin Control Controller
CTRL_SC.K   = 1e-02;         % [s^-2]
CTRL_SC.K_1 = 5e-03;         % [ ] 
CTRL_SC.K_2 = 5e-03;         % [kg.m^-2] 
CTRL_SC.vec_tgt = vnorm([1;0;0]);   % [-] Desired spin vector in Body Frame
CTRL_SC.w_tgt   = 0.01;      % [rad/s] Desired Angular Velocity
CTRL_SC.type    = 'hihb';    % [ ] - 'ruiter','buhl','hihb'


% Controller Panel
CTRL_SWITCH1  = P/8; % Time to change mode from Detumbling to Sun Pointing.
CTRL_SWITCH2  = P/4; % Time to change mode from Sun Pointing to Operation Mode.

CTRL_DT_MODE  = 2;   % Detumbling mode: BDOT/EDSP/VDOT/IDLE
CTRL_PT1_MODE = 3;   % Pointing mode  : REF/SLD/SUN/TA/SC/IDLE
CTRL_PT2_MODE = 2;   % Pointing mode  : REF/SLD/SUN/TA/SC/IDLE

%% SOLVER
CONST.model = 'satellite_adcs_model';
tdur = 3*P/8;            
sim('satellite_adcs_model',tdur);


%% POST PROCESSING
R     = R/CONST.Re;     % Position Vector of satellite (Normalized)
R_tgt = R_tgt/CONST.Re; % Target Vector of satellite (Normalized)
 
for i=1:1:length(tout)
Rx(i) = R(1,1,i); % Position Vector X of spacecraft
Ry(i) = R(2,1,i); % Position Vector Y of spacecraft
Rz(i) = R(3,1,i); % Position Vector Z of spacecraft
    
vr(i) = norm(Vr(:,1,i));
vt(i) = norm(Vt(:,1,i));
v(i)  = sqrt(vr(i)^2+vt(i));

% EKF
q_B_I_error(:,i)  = q_B_I_f(:,i) - q_B_I(:,i);
e_B_I_error(:,i)  = e_B_I_f(:,i) - e_B_I(:,i);
w_B_BI_error(:,i) = w_B_BI_f(:,i) - w_B_BI(:,i);
w_B_BI_m_error(:,i) = w_B_BI_m(:,i)-w_B_BI(:,i)-bias_f(:,i);
bias_error(:,i)   = bias_f(:,i) - bias(:,i);

q_B_I_error_ukf(:,i)  = q_B_I_ukf(:,i) - q_B_I(:,i);
e_B_I_error_ukf(:,i)  = e_B_I_ukf(:,i) - e_B_I(:,i);
w_B_BI_error_ukf(:,i) = w_B_BI_ukf(:,i) - w_B_BI(:,i);
bias_error_ukf(:,i)   = bias_ukf(:,i) - bias(:,i);

q_B_I_error_skf(:,i)  = q_B_I_skf(:,i) - q_B_I(:,i);
e_B_I_error_skf(:,i)  = e_B_I_skf(:,i) - e_B_I(:,i);
w_B_BI_error_skf(:,i) = w_B_BI_skf(:,i) - w_B_BI(:,i);
bias_error_skf(:,i)   = bias_skf(:,i) - bias(:,i);

Pdiag(1,i)        = Pk_skf(1,1,i);
Pdiag(2,i)        = Pk_skf(2,2,i);
Pdiag(3,i)        = Pk_skf(3,3,i);
Pdiag(4,i)        = Pk_skf(4,4,i);
Pdiag(5,i)        = Pk_skf(5,5,i);
Pdiag(6,i)        = Pk_skf(6,6,i);

R_B_I_m_1(:,:,i) = q2dcm(q_B_I_m_1(:,:,i));
R_B_I_m_2(:,:,i) = q2dcm(q_B_I_m_2(:,:,i));
R_B_I_m_3(:,:,i) = q2dcm(q_B_I_m_3(:,:,i));

R_B_I_tgt(:,:,i) = q2dcm(q_B_I_tgt(:,:,i));
R_B_I_f(:,:,i)   = q2dcm(q_B_I_f(:,:,i));
R_B_I_ukf(:,:,i) = q2dcm(q_B_I_ukf(:,:,i));
R_B_I_skf(:,:,i) = q2dcm(q_B_I_skf(:,:,i));

% Measured Star Tracker
LOS_error_1(1,i)  = rad2arcsec(vangle(R_B_I_m_1(:,:,i)'*[1;0;0],R_B_I(:,:,i)'*[1;0;0]));
LOS_error_1(2,i)  = rad2arcsec(vangle(R_B_I_m_1(:,:,i)'*[1;0;0],R_B_I(:,:,i)'*[1;0;0]));
LOS_error_1(3,i)  = rad2arcsec(vangle(R_B_I_m_1(:,:,i)'*[1;0;0],R_B_I(:,:,i)'*[1;0;0]));

% Measured Star Tracker
LOS_error_3(1,i)  = rad2arcsec(vangle(R_B_I_m_3(:,:,i)'*[1;0;0],R_B_I(:,:,i)'*[1;0;0]));
LOS_error_3(2,i)  = rad2arcsec(vangle(R_B_I_m_3(:,:,i)'*[1;0;0],R_B_I(:,:,i)'*[1;0;0]));
LOS_error_3(3,i)  = rad2arcsec(vangle(R_B_I_m_3(:,:,i)'*[1;0;0],R_B_I(:,:,i)'*[1;0;0]));

% EKF
LOS_error(1,i)  = rad2arcsec(vangle(R_B_I_f(:,:,i)'*[1;0;0],R_B_I(:,:,i)'*[1;0;0]));
LOS_error(2,i)  = rad2arcsec(vangle(R_B_I_f(:,:,i)'*[0;1;0],R_B_I(:,:,i)'*[0;1;0]));
LOS_error(3,i)  = rad2arcsec(vangle(R_B_I_f(:,:,i)'*[0;0;1],R_B_I(:,:,i)'*[0;0;1]));

% UKF
LOS_error_ukf(1,i)  = rad2arcsec(vangle(R_B_I_ukf(:,:,i)'*[1;0;0],R_B_I(:,:,i)'*[1;0;0]));
LOS_error_ukf(2,i)  = rad2arcsec(vangle(R_B_I_ukf(:,:,i)'*[0;1;0],R_B_I(:,:,i)'*[0;1;0]));
LOS_error_ukf(3,i)  = rad2arcsec(vangle(R_B_I_ukf(:,:,i)'*[0;0;1],R_B_I(:,:,i)'*[0;0;1]));

% SKF
LOS_error_skf(1,i)  = rad2arcsec(vangle(R_B_I_skf(:,:,i)'*[1;0;0],R_B_I(:,:,i)'*[1;0;0]));
LOS_error_skf(2,i)  = rad2arcsec(vangle(R_B_I_skf(:,:,i)'*[0;1;0],R_B_I(:,:,i)'*[0;1;0]));
LOS_error_skf(3,i)  = rad2arcsec(vangle(R_B_I_skf(:,:,i)'*[0;0;1],R_B_I(:,:,i)'*[0;0;1]));



LOS_SUN(i)     = rad2arcsec(vangle(S_I(:,i), R_B_I(:,:,i)'*[1 ;0 ;0]));
LOS_NADIR(i)   = rad2arcsec(vangle(R_O_I(:,:,i)'*[0 ;0 ;1], R_B_I(:,:,i)'*[0 ;0 ;1]));

LOS_TGT(i)     = rad2arcsec(vangle(R_B_I_tgt(:,:,i)'*CONST.los_vec,R_B_I(:,:,i)'*CONST.los_vec));
LOS_TGT_A(i)   = vangle(R_B_I_tgt(:,:,i)'*CONST.los_vec,R_B_I(:,:,i)'*CONST.los_vec);
LOS_ANGLE(i)   = norm(cross(R_B_I_tgt(:,:,i)'*CONST.los_vec,R_B_I(:,:,i)'*CONST.los_vec));
end

%% PLOT
satellite_adcs_plot

%% SIMULATION

createSimulation([0 0 1 1],2)

% Earth Centered Fixed Frame
[X_ecf,X_ecf_lab] = plotvector(R_I_E(:,:,1)*[1 ;0 ;0], [0 0 0], 'r','X_ecf');
[Y_ecf,Y_ecf_lab] = plotvector(R_I_E(:,:,1)*[0 ;1 ;0], [0 0 0], 'r','Y_ecf');
[Z_ecf,Z_ecf_lab] = plotvector(R_I_E(:,:,1)*[0 ;0 ;1], [0 0 0], 'r','Z_ecf');

% Earth Centered Magnetic Frame
% X_emf = plotvector(R_I_E(:,:,1)*R_E_M*[1 ;0 ;0], [0 0 0], 'm');
% Y_emf = plotvector(R_I_E(:,:,1)*R_E_M*[0 ;1 ;0], [0 0 0], 'm');
% Z_emf = plotvector(R_I_E(:,:,1)*R_E_M*[0 ;0 ;1], [0 0 0], 'm');

% Satellite Body Frame
[X_sat,X_sat_lab] = plotvector(R_B_I(:,:,1)'*[1 ;0 ;0], R(:,1,1), 'b', 'x_b',0.5);
[Y_sat,Y_sat_lab] = plotvector(R_B_I(:,:,1)'*[0 ;1 ;0], R(:,1,1), 'b', 'y_b',0.5);
[Z_sat,Z_sat_lab] = plotvector(R_B_I(:,:,1)'*[0 ;0 ;1], R(:,1,1), 'b', 'z_b',0.5);
[w_sat,w_sat_lab] = plotvector(R_B_I(:,:,1)'*w_B_BI(:,1), R(:,1,1),'r','\omega',0.5);
[B_sat,B_sat_lab] = plotvector(R_B_I(:,:,1)'*B_B_m(:,1), R(:,1,1),'m','B_m',0.5);
[S_sat,S_sat_lab] = plotvector(-R_B_I(:,:,1)'*S_B_hat(:,1), R(:,1,1),color('orange'),'S_m',0.5);

% Sun Sensor Frame
for j = 1:SSnum
[Z_ss(j),Z_ss_lab(j)] = plotvector(R_B_I(:,:,1)'*R_S_B(:,:,j)'*[0 ;0 ;1], R(:,1,1), color('orange'), strcat('SS',num2str(j)),0.5);
end
% Orbital Frame
[X_orb,X_orb_lab] = plotvector(R_O_I(:,:,1)'*[1 ;0 ;0], R(:,1,1), 'g', 'x_o',0.5);
[Y_orb,Y_orb_lab] = plotvector(R_O_I(:,:,1)'*[0 ;1 ;0], R(:,1,1), 'g', 'y_o',0.5);
[Z_orb,Z_orb_lab] = plotvector(R_O_I(:,:,1)'*[0 ;0 ;1], R(:,1,1), 'g', 'z_o',a/CONST.Re);

% Magnetic Field 
[B_mag,B_mag_lab] = plotvector(B_I(:,1), R(:,1,1), 'm', 'B',0.25);

% Sun Vector
[S_sun, S_sun_lab] = plotvector(S_I(:,1), [0;0;0], 'r', 'Sun',2);
[S_hat, S_hat_lab] = plotvector(-S_I_hat(:,1), [0;0;0], color('orange'), 'S_h_a_t',2);

% Eclipse Status
eclipse_lab = text(1.5,1.5,1.5,strcat('Eclipse:',int2str(ECLIPSE(1))));

% Velocity Vector
Vplot  = plotvector(V(:,1,1), R(:,1,1), 'r');  % Velocity Vector

% Satellite Position
R_sat = plotposition(R,'b','s');
hold on;

% Plot Satellite Orbit Track
plot3(Rx,Ry,Rz,'-.')

[R_target,R_target_lab] = plotposition(R ,'r','s','tgt');


% Target Frame
[X_tgt,X_tgt_lab] = plotvector(R_B_I_tgt(:,:,1)'*[1;0;0],R,color('teal'),'x_t_g_t',0.75);
[Y_tgt,Y_tgt_lab] = plotvector(R_B_I_tgt(:,:,1)'*[0;1;0],R,color('teal'),'y_t_g_t',0.75);
[Z_tgt,Z_tgt_lab] = plotvector(R_B_I_tgt(:,:,1)'*[0;0;1],R,color('teal'),'z_t_g_t',0.75);


% MAGNETIC FIELD PLOT
%  MagneticField();

% UPDATE SIMULATION PLOT

for i=1:1:(length(tout)-1)

% Update Satellite Position
updateposition(R_sat, R(:,i));

% Update ECF Frames
updatevector(X_ecf, R_I_E(:,:,i)*[1 ;0 ;0], [0 0 0]);
updatevector(Y_ecf, R_I_E(:,:,i)*[0 ;1 ;0], [0 0 0]);
updatevector(Z_ecf, R_I_E(:,:,i)*[0 ;0 ;1], [0 0 0]);
set(X_ecf_lab,'Position',R_I_E(:,:,i)*[1 ;0 ;0]);
set(Y_ecf_lab,'Position',R_I_E(:,:,i)*[0 ;1 ;0]);
set(Z_ecf_lab,'Position',R_I_E(:,:,i)*[0 ;0 ;1]);

% Update EMF Frames
% updatevector(X_emf, R_I_E(:,:,i)*R_E_M*[1 ;0 ;0], [0 0 0]);
% updatevector(Y_emf, R_I_E(:,:,i)*R_E_M*[0 ;1 ;0], [0 0 0]);
% updatevector(Z_emf, R_I_E(:,:,i)*R_E_M*[0 ;0 ;1], [0 0 0]);

% Update Earth Centered Sun Frame
updatevector(S_sun, S_I(:,i),  [0 0 0],2);

% Update Earth Centered Sun Frame
updatevector(S_hat, -S_I_hat(:,i),  [0 0 0],2);

% Update Orbit Frame
updatevector(X_orb, R_O_I(:,:,i)'*[1 ;0 ;0],  R(:,1,i),0.5);
updatevector(Y_orb, R_O_I(:,:,i)'*[0 ;1 ;0],  R(:,1,i),0.5);
updatevector(Z_orb, R_O_I(:,:,i)'*[0 ;0 ;1],  R(:,1,i),a/CONST.Re);
set(X_orb_lab,'Position',R_O_I(:,:,i)'*[1 ;0 ;0]+R(:,1,i));
set(Y_orb_lab,'Position',R_O_I(:,:,i)'*[0 ;1 ;0]+R(:,1,i));
set(Z_orb_lab,'Position',R_O_I(:,:,i)'*[0 ;0 ;1]+R(:,1,i));

% Update Satellite Body Frame
updatevector(X_sat, R_B_I(:,:,i)'*[1 ;0 ;0],  R(:,1,i),0.5);
updatevector(Y_sat, R_B_I(:,:,i)'*[0 ;1 ;0],  R(:,1,i),0.5);
updatevector(Z_sat, R_B_I(:,:,i)'*[0 ;0 ;1],  R(:,1,i),0.5);
updatevector(w_sat, R_B_I(:,:,i)'*w_B_BI(:,i),  R(:,1,i),0.5);
updatevector(B_sat, R_B_I(:,:,i)'*B_B_m(:,i),  R(:,1,i),0.5);
updatevector(S_sat, -R_B_I(:,:,i)'*S_B_hat(:,i),  R(:,1,i),0.5);


set(X_sat_lab,'Position',R_B_I(:,:,i)'*0.5*[1 ;0 ;0]+R(:,1,i));
set(Y_sat_lab,'Position',R_B_I(:,:,i)'*0.5*[0 ;1 ;0]+R(:,1,i));
set(Z_sat_lab,'Position',R_B_I(:,:,i)'*0.5*[0 ;0 ;1]+R(:,1,i));
set(w_sat_lab,'Position',R_B_I(:,:,i)'*0.5*vnorm(w_B_BI(:,i))+R(:,1,i));
set(B_sat_lab,'Position',R_B_I(:,:,i)'*0.5*vnorm(B_B_m(:,i))+R(:,1,i));
set(S_sat_lab,'Position',-R_B_I(:,:,i)'*0.5*vnorm(S_B_hat(:,i))+R(:,1,i));


% Sun Sensor
for j = 1:SSnum
updatevector(Z_ss(j), R_B_I(:,:,i)'*R_S_B(:,:,j)'*[0 ;0 ;1],  R(:,1,i),0.5);
set(Z_ss_lab(j),'Position',R_B_I(:,:,i)'*R_S_B(:,:,j)'*[0 ;0 ;0.5]+R(:,1,i));
end




updatevector(X_tgt, R_B_I_tgt(:,:,i)'*[1 ;0 ;0],  R(:,1,i),0.75);
updatevector(Y_tgt, R_B_I_tgt(:,:,i)'*[0 ;1 ;0],  R(:,1,i),2.50);
updatevector(Z_tgt, R_B_I_tgt(:,:,i)'*[0 ;0 ;1],  R(:,1,i),0.75);
set(X_tgt_lab,'Position',R_B_I_tgt(:,:,i)'*0.75*[1 ;0 ;0]+R(:,1,i));
set(Y_tgt_lab,'Position',R_B_I_tgt(:,:,i)'*0.75*[0 ;1 ;0]+R(:,1,i));
set(Z_tgt_lab,'Position',R_B_I_tgt(:,:,i)'*0.75*[0 ;0 ;1]+R(:,1,i));

% Update Target
updateposition(R_target, R_tgt(:,i));
set(R_target_lab,'Position',R_tgt(:,i));


% Update Magnetic Frame
updatevector(B_mag, B_I(:,i),  R(:,1,i));

% Update Eclipse
set(eclipse_lab,'String',strcat('Eclipse:',int2str(ECLIPSE(i))));

% Update Velocity Vectors
updatevector(Vplot, V(:,1,i), R(:,1,i));

drawnow;  
end

