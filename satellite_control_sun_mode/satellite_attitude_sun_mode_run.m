%% P6E1_RUN
% Do take note that the inertia frame is orbit frame.
clear 
close all
clc

global I_sc

r2d = 180/pi;
%% ADD PATH
addpath(genpath(cd)) % Initialises the paths (to the support functions) and provides any other common initialisation
%% INITIALISATION
% Quaternion initialisaion
e     = [0 ;1 ;0];                       % [-] euler axis at start of simulation describing rotation from inertial to body frame
phi   = 45/r2d;                          % [r] euler angle at start of simulation describing rotation from inertial to body frame
q_bi0 = [e*sin(phi/2); cos(phi/2)];      % [-] initial inertial to body attitude quaternion
w_bi0 = [0.00;0.0;0.0];                   % [rad/s] initial spacecraft inertial rate in body frame

T_env = [0.1 0.1 0.1]';                 % [Nm] environmental disturbance torque
I_sc = diag([10000, 30000, 30000]);     % [kgm^2] spacecraft inertia matrix

s_i     = [0 0 1]';                            % [-] sun vector in inertial frame (fixed)

%% CONTROLLER PARAMETERS
tdelay  = 0.2;                                 % [sec] transport delay 
ta      = 0.5;                                 % [sec] controller update time (i.e. interval at which controller output is updated)
Kpx     = 125;                                 % [Nm/r] proportional gain for spacecraft X-axis controller
Kdx     = 5000;                                % [Nm/(r/s)] derivative gain for spacecraft X-axis controller
Kpy     = 375;                                 % [Nm/r] proportional gain for spacecraft Y-axis controller
Kdy     = 15000;                               % [Nm/(r/s)] derivative gain for spacecraft Y-axis controller
Kdz     = 1150;                                % [Nm/(r/s)] proportional gain for spacecraft Z-axis (rate) controller

%% ATTITUDE DEMANDS 
alpha_dem = 0;                                 % [r] input demand to X-axis controller, corresponds to Z-axis pointing at the Sun
beta_dem  = 0;                                 % [r] input demand to X-axis controller, corresponds to Z-axis pointing at the Sun

%% RATE DEMANDS
rate_dem_x = 0;
rate_dem_y = 0;
rate_dem_z = 0;

%% SENSORS PARAMETERS
w_bias                 = [0 0 -0.1]/r2d;       % [r/s] rate sensor bias (sensor output when actual rate=0)
sigma_w                = 0.0005/r2d;           % [r/s] rates sensor read-out noise standard deviation
sun_sensor_model_type  = 1;                    % [-] sun sensor model type flag (1=perfect sensor , 2=non-linear (as described in lecture notes) sensor)
rate_sensor_model_type = 1;                    % [-] rate sensor model type flag (1=perfect sensor with delay, 2 = noisy sensor with delay with bias)

%% PWM & RCS ACTUATORS
Tdb                    = [2 5 2];              % [Nm] PWM torque deadbands for [X Y Z] axes respectively
Trcs_saturation        = [12 58 57];           % [Nm] Maximum control torques available per axis [X Y Z] from RCS 
Krcs                   = [1 1 1];              % [-] RCS gain


%% RUN SIMULATION
dt   = 0.1;
tdur = 200;
sim('satellite_attitude_sun_mode_model',tdur)

%% POST-PROCESSING
t = tout';
s_b         = s_b';
q_bi        = q_bi';
w_bi        = w_bi';
w_bi_meas   = w_bi_meas';
alpha_error = alpha_error';
beta_error  = beta_error';
wz_error    = wz_error';

% Attitude Deadbands from Torque Deadbands
attitude_deadband_x = Tdb(1)/Kpx;
attitude_deadband_y = Tdb(2)/Kpy;
rate_deadband_z     = Tdb(3)/Kdz;


%% PLOTS
close all
satellite_attitude_sun_mode_plot

%% LOCUS PLOTTING
npts = length(t);
for i=1:npts
    q_bi(:,i)
    DCM         = q2dcm(q_bi(:,i))';    % obtain the DCM for each time step
    zb_eci(:,i) = DCM * [0 0 1]';       % transform axis in body frame to inertial frame
end

%% PLOT3D SPACECRAFT INERTIAL FRAME SIMULATION
Vis3DAnimateAttitude(tout',q_bi, 10, 'Satellite', 2, zb_eci)