% This script simulates the rate controller response about the Z
% axes for the ESM mode design described in the p6 notes.


% -------------------------------------------------------------------------
% Preliminaries
% -------------------------------------------------------------------------
clear all
close all
r2d            = 180/pi;

% -------------------------------------------------------------------------
% Simulator initialisation & configuration parameters
% -------------------------------------------------------------------------
tdur           = 500;                       % duration of simulation
dt             = 0.1;                       % time step used in simulation
w0             = 1*pi/180;                  % initial rate error

% -------------------------------------------------------------------------
% Set control loop parameters from lecture notes (for Z axis)
% -------------------------------------------------------------------------
I              = 30000;                     % axis moment of inertia
Kd             = 1150;                      % axis Kd (=derivative) gain
Trcs_max       = 57;                        % maximum control torques available 
Tdist          = 0.1;                       % external disturbance torquess
Krcs           = 1;                         % RCS gain
t_delay        = 3;                         % overall loop time delay 

% -------------------------------------------------------------------------
% Run simulink - no RCS saturation
% -------------------------------------------------------------------------
sim('rate_mdl',tdur)
t              = tout;                      % Simulink automatically returns simulation time series to the workspace, this line simply assigns it to a shorter variable name

% -------------------------------------------------------------------------
% Post-processing / predictions
% -------------------------------------------------------------------------
rate_ss        = Tdist/Kd;                  % steady state rate error expected given disturbance torque

% -------------------------------------------------------------------------
% Plot results - no RCS saturation
% -------------------------------------------------------------------------
figure
plot(t,w*r2d), hold on, grid on, zoom on
plot([0 tdur], rate_ss*[1 1]*r2d,'r--'), grid on, zoom on
xlabel('time [sec]')
ylabel(['Z rate [deg/sec]'])

figure
plot(t,Trcs), hold on, grid on, zoom on
plot([0 tdur], +Trcs_max*[1 1],'r--')
plot([0 tdur], -Trcs_max*[1 1],'r--')
xlabel('time [sec]')
ylabel(['Z torque demand [Nm]'])

% -------------------------------------------------------------------------
% Run simulink - with RCS saturation & add overlays to existing graphs
% -------------------------------------------------------------------------
sim('rate_saturator_mdl',tdur)

figure(1)
plot(t,w2*r2d,'m')

figure(2)
plot(t,Trcs2,'m')

% -------------------------------------------------------------------------
% Run simulink - with RCS saturation & delay & add overlays to existing graphs
% -------------------------------------------------------------------------
sim('rate_delay_mdl',tdur)

figure(1)
plot(t,w3*r2d,'g')

figure(2)
plot(t,Trcs3,'g')
