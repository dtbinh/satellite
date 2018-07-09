clc
clear 
close all

%% PARAMETERS FOR EARTH AND ORBIT

global m
global I
global w_O
global Rc
global B_null
global K_coil
global K_p
global K_d
global w_O_IO

m  = 2.60;  % [kg] Satellite Mass
dx = 0.10;  % [m] Length X
dy = 0.10;  % [m] Length Y
dz = 0.20;  % [m] Length Z

Re = 6371e3; % [m] Earth Radius
Rs = 600e3;  % [m] Satellite Altitude
Rc = Re+Rs;  % [m] Distance from Earth Center to Satellite

G  = 6.67428e-11;  % [m^3/kg/s^2] Earth Gravitational Constant
M  = 5.972e24;     % [kg] Earth Mass

w_O    = sqrt(G*M/Rc^3); % [rad/s] Angular Velocity of Orbit Relative to Earth
w_O_IO = [0 -w_O 0]';    % [rad/s] Angular Velocity of Orbit Frame to Inertia in Orbit Frame

orbitPeriod = 2*pi/w_O;  % [s] Orbital Period 
%% MOMENTS OF INERTIA

Ix = (m/12)*(dy^2+dz^2); % [kg.m^2] X-axis Inertia
Iy = (m/12)*(dx^2+dz^2); % [kg.m^2] Y-axis Inertia
Iz = (m/12)*(dx^2+dy^2); % [kg.m^2] Z-axis Inertia

I  = diag([Ix Iy Iz]);   % [kg.m^2] Inertia Matrix


%% INITIAL ORIENTATION of body frame in orbit frame

rol_0 = 10/180*pi; % [rad] initial roll angle
pit_0 =  5/180*pi; % [rad] initial pitch angle
yaw_0 = -2/180*pi; % [rad] initial yaw angle

q_0 = euler2q(rol_0,pit_0,yaw_0); % Quaternions from Euler by Rotation Matrix Rzxy of euler angles
    
%% INITIAL ANGULAR VELOCITY
w_B_OB_0 = [0.1; -0.2; 0.05]; % [rad/s] Initial angular velocity of satellite relative to Orbit Frame expressed in Body Frame

R_O_B = Rquat(q_0);          % [3x3] Rotation Matrix from body frame to orbit frame. % R = Rquat(q) computes the quaternion rotation matrix R in SO(3) for q = [eta eps1 eps2 eps3]
R_B_O = R_O_B';              % [3x3] Rotation Matrix from orbit frame to body frame.

w_B_IB_0 = w_B_OB_0+R_B_O*w_O_IO;   % [rad/s] Angular Velocity of Satellite to Inertia Frame expressed in Body Frame

%% REGULATOR GAINS

K_p = 5e-08; % [-] Proportional Gain
K_d = 4e-05; % [-] Differential Gain

%% COIL PARAMETERS AND CONTROL ALLOCATION MATRIX

Nx = 355; % Number of turns with copper thread in coil X
Ny = 355; % Number of turns with copper thread in coil Y
Nz = 800; % Number of turns with copper thread in coil Z

Ax = 0.08*0.18; % Area of Coil X
Ay = 0.08*0.18; % Area of Coil Y
Az = 0.08*0.08; % Area of Coil Z

Rx = 110; % Resistance in Coil X
Ry = 110; % Resistance in Coil Y
Rz = 110; % Resistance in Coil Z

% K Coil Matrix
K_coil = [Nx*Ax/Rx   0        0        ;
             0       Ny*Ay/Ry 0        ;
             0       0        Nz*Az/Rz ];
    
%% DIPOLE MAGNETIC FIELD OF EARTH

my      = 1e17/(4*pi);   % Permeability of vacuum
B_null  = my/(Rc^3);     % Magnitude of magnetic field 

imu_sigma = 2e-4;          % Magnetometer read-out noise standard deviation

%% SIMULATION
dt   = 1;
tdur = 5*orbitPeriod;
sim('satellite_control_magnetorquer_model',tdur);

%% POST PROCESSING

w_b_ob(:,1) = w_B_OB(1,:,:);
w_b_ob(:,2) = w_B_OB(2,:,:);
w_b_ob(:,3) = w_B_OB(3,:,:);
w_b_ob = w_b_ob';

w_b_ib = w_B_IB;


phi   = zeros(length(q),1);
theta = zeros(length(q),1);
psi   = zeros(length(q),1);

for i=1:1:length(q)
q(:,i)    = q(:,i)./norm(q(:,i));
[phi(i,1),theta(i,1),psi(i,1)]      = q2euler(q(:,i));

end
%% SIMULATION PLOT
%  Satellite Attitude
figure
subplot(3,1,1)
plot(tout,phi/pi*180)
grid on; axis on;
xlabel('Time [s]');
ylabel('\phi [\circ]');

subplot(3,1,2)
plot(tout,theta/pi*180)
grid on; axis on;
xlabel('Time [s]');
ylabel('\theta [\circ]');

subplot(3,1,3)
plot(tout,psi/pi*180)
grid on; axis on;
xlabel('Time [s]');
ylabel('\psi [\circ]');


%% SIMULATION PLOT
%  Angular Velocity Relative to Orbit Frame
figure
subplot(3,1,1)
plot(tout,w_b_ob(1,:))
grid on; axis on;
xlabel('Time [s]');
ylabel('\omega^b_{ob} [rad/s] ');
title('Angular Velocity Relative to Orbit Frame')

subplot(3,1,2)
plot(tout,w_b_ob(2,:))
grid on; axis on;
xlabel('Time [s]');
ylabel('\omega^b_{ob} [rad/s] ');

subplot(3,1,3)
plot(tout,w_b_ob(3,:))
grid on; axis on;
xlabel('Time [s]');
ylabel('\omega^b_{ob} [rad/s] ');
%% SIMULATION PLOT
%  Angular Velocity Relative to Inertia Frame
figure
subplot(3,1,1)
plot(tout,w_b_ib(1,:))
grid on; axis on;
xlabel('[s] Time');
ylabel('\omega^b_{ib} [rad/s]');
title('Angular Velocity Relative to Inertia Frame')

subplot(3,1,2)
plot(tout,w_b_ib(2,:))
grid on; axis on;
xlabel('Time [s]');
ylabel('\omega^b_{ib} [rad/s]');

subplot(3,1,3)
plot(tout,w_b_ib(3,:))
grid on; axis on;
xlabel('Time [s]');
ylabel('\omega^b_{ib} [rad/s] ');

%% SIMULATION PLOT
%  Earth's Magnetic Field in Orbit Frame and Body Frame
figure
subplot(2,1,1)
plot(tout,B_B(1,:))
hold all;grid on; axis on;
plot(tout,B_B(2,:))
plot(tout,B_B(3,:))
xlabel('Time [s]');
ylabel('B_B [T]');

subplot(2,1,2)
plot(tout,B_O(1,:))
hold all;grid on; axis on;
plot(tout,B_O(2,:))
plot(tout,B_O(3,:))
xlabel('Time [s]');
ylabel('B_O [T]');

%% SIMULATION PLOT
%  Angular Velocity Relative to Inertia Frame
figure
subplot(3,1,1)
plot(tout,V_out(1,:))
grid on; axis on;
xlabel('Time [s]');
ylabel('V_x [V]');
title('Voltage Supplied in Magnetic Coil')

subplot(3,1,2)
plot(tout,V_out(2,:))
grid on; axis on;
xlabel('Time [s]');
ylabel('V_y [V]');

subplot(3,1,3)
plot(tout,V_out(3,:))
grid on; axis on;
xlabel('Time [s]');
ylabel('V_z [V] ');