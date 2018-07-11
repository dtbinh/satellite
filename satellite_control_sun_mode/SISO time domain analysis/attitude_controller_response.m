% This script simulates the attitude controller response about the X, Y
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
ax             = 2;                         % axis to analyse (=1 for X or 2 for Y)
tdur           = 500;                       % duration of simulation
dt             = 0.1;                       % time step used in simulation
angle0         = 45*pi/180;                 % initial attitude error
w0             = 1*pi/180;                  % initial rate error

% -------------------------------------------------------------------------
% Set control loop parameters from lecture notes (for [X, Y] axes)
% -------------------------------------------------------------------------
I_array        = [10000 30000];             % axis moments of inertia
Kp_array       = [125 375];                 % axis Kp (=proportional) gains (=0.0125*I_array)
Kd_array       = [5000 15000];              % axis Kd (=derivative) gains (= 0.5*I_array)
Trcs_max_array = [12 58];                   % maximum control torques available 
Tdist_array    = 0*[0.1 0.1];               % external disturbance torques
Krcs           = 1;                         % RCS gain
t_delay        = 10;                       % overall loop time delay 

% -------------------------------------------------------------------------
% Assign parameters based on axis selection
% -------------------------------------------------------------------------
I              = I_array(ax);
Kp             = Kp_array(ax);
Kd             = Kd_array(ax);
Trcs_max       = Trcs_max_array(ax);
Tdist          = Tdist_array(ax);

% -------------------------------------------------------------------------
% Run simulink - no RCS saturation
% -------------------------------------------------------------------------
sim('attitude_mdl',tdur)
t              = tout;                      % Simulink automatically returns simulation time series to the workspace, this line simply assigns it to a shorter variable name

% -------------------------------------------------------------------------
% Post-processing / predictions
% -------------------------------------------------------------------------
att_ss         = Tdist/Kp;                  % steady state attitude error expected given disturbance torque


% -------------------------------------------------------------------------
% Plot results - no RCS saturation
% -------------------------------------------------------------------------
st_axis        = ['X' 'Y'];

figure
h = plot(t,-angle*r2d);
set(h,'LineWidth',2)
hold on, grid on, zoom on
plot([0 tdur], -att_ss*[1 1]*r2d,'r--')
xlabel('time [sec]')
ylabel([st_axis(ax) ' attitude error [deg]'])
ax_bounds = axis;
axis([ax_bounds(1:2) -60 0])

figure
h = plot(t,-w*r2d);
set(h,'LineWidth',2)
hold on, grid on, zoom on
xlabel('time [sec]')
ylabel([st_axis(ax) ' rate error [deg/sec]'])

figure
h = plot(t,Trcs);
set(h,'LineWidth',2)
hold on, grid on, zoom on
plot([0 tdur], +Trcs_max*[1 1],'r--')
plot([0 tdur], -Trcs_max*[1 1],'r--')
xlabel('time [sec]')
ylabel([st_axis(ax) ' torque demand [Nm]'])

% -------------------------------------------------------------------------
% Run simulink - with RCS saturation & add overlays to existing graphs
% -------------------------------------------------------------------------
sim('attitude_saturator_mdl',tdur)

figure(1)
h = plot(t,-angle2*r2d,'r');
set(h,'LineWidth',2)

figure(2)
h = plot(t,-w2*r2d,'r');
set(h,'LineWidth',2)

figure(3)
h = plot(t,Trcs2,'r');
set(h,'LineWidth',2)

% -------------------------------------------------------------------------
% Run simulink - with RCS saturation & delay & add overlays to existing graphs
% -------------------------------------------------------------------------
sim('attitude_delay_mdl',tdur)

figure(1)
h = plot(t,-angle3*r2d,'g');
set(h,'LineWidth',2)
legend('base','Trcs_m_i_n _m_a_x','sat','sat + delay');

figure(2)
h = plot(t,-w3*r2d,'g');
set(h,'LineWidth',2);
legend('base','sat','sat + delay');

figure(3)
h = plot(t,Trcs3,'g');
set(h,'LineWidth',2)
legend('base','Trcs_m_i_n','Trcs_m_a_x','sat','sat + delay');