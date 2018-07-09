close all
clear 
clc
%% SAMPLE TIME
dt_c = 0.05;   % [s] controller discrete step
dt_w = 0.05;   % [s] motor discrete step
dt   =   0.001;
%% INITIALISATION
% DCM from Wheel Frame to Body Frame
DCM_b_w = [ 0.40825 0.40825 -0.81650;
           -0.70711 0.70711  0.00000;
            0.57735 0.57735  0.57735];

% Moments of Inertia
Ixx = 4.0774;
Iyy = 4.1360;
Izz = 2.3216;

I_b = diag([Ixx Iyy Izz]);

Ia  = 0.00725;
I_w = diag([Ia Ia Ia]);

%Linearlized State Space Model of AirBearing
[Ad_b,Bd_b,Cd_b]= linear_eq(Ixx,Iyy,Izz,Ia,DCM_b_w,dt_w);

RW1_init = 40;
RW2_init = 40;
RW3_init = -80;

x_pos_init = 0;
y_pos_init = 0;
z_pos_init = 0;

x_rate_init = 0;
y_rate_init = 0;
z_rate_init = 0;

DCM_b_i_0 = eul2qua(x_pos_init,y_pos_init,z_pos_init); % DCM from Inertia Frame to Body Frame

theta_0 = [x_pos_init ;y_pos_init ;z_pos_init];     % [rad/s] Initial Angular Orientation of Body
omega_0 = [x_rate_init ;y_rate_init ;z_rate_init];  % [rad/s] Initial Angular Velocity of Body
speed_0 = [RW1_init ;RW2_init ;RW3_init];           % [rad/s] Initial Angular Velocity of Wheel
state_0 = [theta_0;omega_0];

%% MOTOR PARAMETERS
K_t = 0.070;   % [Nm/A] Torque Constant
K_e = 0.071;   % [V/(rad/s)] Voltage Constant
R   = 1.52 ;   % [Ohm] Resistance
L   = 0.00339; % [H] Inductance
b   = 0.000178;% [Nm/(rad/s)] Damping Ratio
J   = 0.00725; % [kgm^2] Inertia

i_0 = [0 ;0 ;0];
v_max = 10;

mag_variance   = [0;0;0];%[0.0086e-6;0.0025e-6;0.0100e-6];
grav_variance  = 0;
omega_variance = [0;0;0];%[1.706e-6;1.843e-6;2.544e-6];
%% INITIAL CONDITIONS TO SIMULATION
mag_ref  = [0 ; 1400 ; 0];
grav_ref = [0 ; 0 ; -1];

% State Space Model for Reaction Wheels
A_rw = [-0.026    0        0       9.6552      0         0        ;
         0       -0.0246   0       0           9.6552    0        ;
         0        0       -0.0246  0           0         9.6552   ;
        -20.9440  0        0      -448.3776    0         0        ;
         0       -20.9440  0       0          -448.3776  0        ;
         0        0       -20.9440 0           0        -448.3776 ];
     
B_rw = [ 0         0        0      ;
         0         0        0      ;
         0         0        0      ;
        294.9853   0        0      ;
         0       294.9853   0      ;
         0         0       294.9853];

C_rw = [ 1  0  0  0  0  0;
         0  1  0  0  0  0;
         0  0  1  0  0  0];
     
D_rw = [ 0  0  0;
         0  0  0;
         0  0  0];

sysc_rw = ss(A_rw, B_rw, C_rw, D_rw);
sysd_rw = c2d(sysc_rw,dt_w);

Ad_rw = sysd_rw.a;
Bd_rw = sysd_rw.b;
Cd_rw = sysd_rw.c;
%% KALMAN FILTER CONSTANTS, LINEAR QUADRATIC ESTIMATOR

Q_lqe = 0.05*eye(6,6);
R_lqe = 0.10*eye(12,12);

C_lqe = [ 0 0 0 0 0 0 ;
          0 0 0 0 0 0 ;
          0 0 0 0 0 0 ;
          0 0 0 1 0 0 ;
          0 0 0 0 1 0 ;
          0 0 0 0 0 1 ;
          1 0 0 0 0 0 ;
          0 1 0 0 0 0 ;
          0 0 1 0 0 0 ;
          1 0 0 0 0 0 ;
          0 1 0 0 0 0 ;
          0 0 1 0 0 0 ];

Lss = dlqe(Ad_b, eye(6,6),C_lqe, Q_lqe,R_lqe);

%% Discrete Linear Quadratic Regulator Constants
Q_lqr = 1*[ 50000   0     0      0      0      0  ;
              0   50000   0      0      0      0  ;
              0     0   50000    0      0      0  ;
              0     0     0    500000   0      0  ;
              0     0     0      0   500000    0  ;
              0     0     0      0      0    500000];
          
R_lqr = 1*[1 0 0;
           0 1 0;
           0 0 1];

Kp = dlqr(Ad_b,Bd_b,Q_lqr,R_lqr);

%% DERIVATIVE GAIN
Kd = 1.5;

%% SIMULATION
tdur = 100;
sim('satellite_attitude_kalman_model',tdur)
%% POST PROCESSING

theta_plot(:,1) = theta(1,:,:);
theta_plot(:,2) = theta(2,:,:);
theta_plot(:,3) = theta(3,:,:);

omega_plot(:,1) = omega(1,:,:);
omega_plot(:,2) = omega(2,:,:);
omega_plot(:,3) = omega(3,:,:);

theta_cmd_plot(:,1) = theta_cmd.data(1,:,:);
theta_cmd_plot(:,2) = theta_cmd.data(2,:,:);
theta_cmd_plot(:,3) = theta_cmd.data(3,:,:);

omega_cmd_plot(:,1) = omega_cmd.data(1,:,:);
omega_cmd_plot(:,2) = omega_cmd.data(2,:,:);
omega_cmd_plot(:,3) = omega_cmd.data(3,:,:);
%% PLOT
%  Angular Orientation
figure
subplot(3,1,1)
plot(tout,theta_plot(:,1))
grid on; hold on;
plot(theta_cmd.time,theta_cmd_plot(:,1),'r')
xlabel('Time [s]');
ylabel('\theta_x [rad] ');
title('Angular Orientation')

subplot(3,1,2)
plot(tout,theta_plot(:,2))
grid on; hold on;
plot(theta_cmd.time,theta_cmd_plot(:,2),'r')
xlabel('Time [s]');
ylabel('\theta_y [rad] ');

subplot(3,1,3)
plot(tout,theta_plot(:,3))
grid on; hold on;
plot(theta_cmd.time,theta_cmd_plot(:,3),'r')
xlabel('Time [s]');
ylabel('\theta_z [rad] ');
%% PLOT
%  Angular Velocity 
figure
subplot(3,1,1)
plot(tout,omega_plot(:,1))
grid on; hold on;
plot(omega_cmd.time,omega_cmd_plot(:,1))
xlabel('Time [s]');
ylabel('\omega_x [rad/s] ');
title('Angular Velocity ')

subplot(3,1,2)
plot(tout,omega_plot(:,2))
grid on; hold on;
plot(omega_cmd.time,omega_cmd_plot(:,2))
xlabel('Time [s]');
ylabel('\omega_y [rad/s] ');

subplot(3,1,3)
plot(tout,omega_plot(:,3))
grid on; hold on;
plot(omega_cmd.time,omega_cmd_plot(:,3))
xlabel('Time [s]');
ylabel('\omega_z [rad/s] ');
