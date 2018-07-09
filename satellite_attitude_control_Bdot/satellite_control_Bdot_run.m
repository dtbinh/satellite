%% NTNU_Design Implementation Attitude Control Magnetic Coil Stabilization (2011)
close all
clear all
clc

%% GLOBAL
global VAR
%% PARAMETERS FOR EARTH AND ORBIT

m  = 2.00;  % [kg] Satellite Mass
dx = 0.10;  % [m] Length X
dy = 0.10;  % [m] Length Y
dz = 0.20;  % [m] Length Z

Re = 6371e3; % [m] Earth Radius
Rs = 600e3;  % [m] Satellite Altitude
Rc = Re+Rs;  % [m] Distance from Earth Center to Satellite

G  = 6.67428e-11;  % [m^3/kg/s^2] Earth Gravitational Constant
M  = 5.972e24;     % [kg] Earth Mass

w_O = sqrt(G*M/Rc^3); % [rad/s] Angular Velocity of Orbit Relative to Earth

%% MOMENTS OF INERTIA

Ix = (m/12)*(dy^2+dz^2); % [kg.m^2] X-axis Inertia
Iy = (m/12)*(dx^2+dz^2); % [kg.m^2] Y-axis Inertia
Iz = (m/12)*(dx^2+dy^2); % [kg.m^2] Y-axis Inertia
  
I  = diag([Ix Iy Iz]);   % [kg.m^2] Inertia 

%% REGULATOR GAINS
K_p = 5e-08; % [-] Proportional Gain
K_d = 4e-05; % [-] Differential Gain
%% COIL PARAMETERS AND CONTROL ALLOCATION MATRIX
V  = 5; % [V] Max voltage

Nx = 355; % Number of turns with copper thread in coil X
Ny = 355; % Number of turns with copper thread in coil Y
Nz = 800; % Number of turns with copper thread in coil Z

Ax = 0.08*0.18; % Area of Coil X
Ay = 0.08*0.18; % Area of Coil Y
Az = 0.08*0.08; % Area of Coil Z

Rx = 110; % Resistance in Coil X
Ry = 110; % Resistance in Coil Y
Rz = 110; % Resistance in Coil Z

ix_max = V/Rx;
iy_max = V/Ry;
iz_max = V/Rz;

% K Coil Matrix
K_coil = [Nx*Ax/Rx   0        0        ;
             0       Ny*Ay/Ry 0        ;
             0       0        Nz*Az/Rz ];
i_max = [V/Rx;V/Ry;V/Rz]; 
%% DIPOLE MAGNETIC FIELD OF EARTH
my          = 1e17/(4*pi);   % Permeability of vacuum
B0          = my/(Rc^3);     % Magnitude of magnetic field 
orbitPeriod = 2*pi/w_O;
scalePlot   = 1/orbitPeriod;

%% SET PARAMETER OBJECT
P.w_O   = w_O;      % [rad/s] Angular Velocity of orbit Frame
P.V     = V;        % [V] Voltage Maximum
P.I     = I;        % [kgm^2] Moment of Inertia
P.K     = K_coil;   % [3x3] K coil Matrix
P.i_max = i_max;    % [A] Ampere Maximum
P.Re    = Re;       % [m] Earth's Radius
P.Rc    = Rc;       % [m] Orbit Frame to Earth Center Distance
P.B0    = B0;       % [T] Magnetic Field
P.K_d   = K_d;      % [-] Controller Gain
P.K_p   = K_p;

P.Nx    = Nx;
P.Ny    = Ny;
P.Nz    = Nz;
P.Ax    = Ax;
P.Ay    = Ay;
P.Az    = Az;
P.ix_max = ix_max;
P.iy_max = iy_max;
P.iz_max = iz_max;

P.orbitPeriod = orbitPeriod;        % [s] Orbital Period
P.scalePlot   = scalePlot;



%% INITIAL ORIENTATION 
% Angle from O Frame to B Frame (rotate using yaw, pit, rol)
% Angle from B Frame to O Frame (rotate using rol,pit,yaw)
rol_0 = 80/180*pi;     % [rad] initial roll angle
pit_0 = -50/180*pi;    % [rad] initial pitch angle
yaw_0 = 170/180*pi;     % [rad] initial yaw angle

[R_O_B_0,q_0] = eul2qua(rol_0,pit_0,yaw_0) % Quaternions from Euler by Rotation Matrix Rzxy of euler angles
% q_0 = [-0.210; 0.373; 0.552; 0.715];
% q_0 = q_0/norm(q_0)
[eul(1),eul(2),eul(3)] = q2euler(q_0);
w_O_IO = [0; -P.w_O; 0];  % [rad/s] Angular Velocity of Orbit Frame to Inertia in Orbit Frame

%% INITIAL ANGULAR VELOCITY
w_B_OB_0 = [0.01; -0.2; 0.03];        % [rad/s] Initial angular velocity of Body to Orbit Frame expressed in Body Frame
R_B_O_0  = R_O_B_0';                 % [] Initial Rotation Matrix - R_B_O - Orbit Frame to Body Frame
w_B_IB_0 = w_B_OB_0+R_B_O_0*w_O_IO;  % [rad/s] Angular Velocity of Body to Inertia Frame expressed in Body Frame

lat_0   = 0; % [rad] Latitude (Initial)
joule_0 = 0; 

%% SIMULATION TIME
numOrbit = 10;                     % [-] Number of Orbits for Simulation
dt       = 1;                      % [s]
tdur     = P.orbitPeriod*numOrbit; % [s]
tSpan    = 0:dt:tdur;              % [s] Time Span for Simulation
%% ODE45 SIMULATION
xInit       = [w_B_IB_0;q_0';lat_0;joule_0];
options     = odeset('Maxstep',1.0,'OutputFcn',@outFcn);%,'RelTol',1e-12,'AbsTol',1e-12,'Refine',1);
[tout,yout] = ode45(@(t,x)nonlinearSatellite(t,x,P),tSpan,xInit,options);

%% SIMULATION
w_B_IB = yout(:,1:3);
q      = yout(:,4:7);
q      = q';

for i = 1:1:length(q)
q(:,i)    = q(:,i)./norm(q(:,i));
[phi(i,1),theta(i,1),psi(i,1)]      = q2euler(q(:,i));

end
%% PLOT ATTITUDE QUATERNION
figure
subplot(4,1,1)
plot(tout,q(1,:))
grid on; axis on;
xlabel('Time [s]');
ylabel('q_0');

subplot(4,1,2)
plot(tout,q(2,:))
grid on; axis on;
xlabel('Time [s]');
ylabel('q_1');

subplot(4,1,3)
plot(tout,q(3,:))
grid on; axis on;
xlabel('Time [s]');
ylabel('q_2');

subplot(4,1,4)
plot(tout,q(4,:))
grid on; axis on;
xlabel('Time [s]');
ylabel('q_3');
%% PLOT ATTITUDE EULER ANGLE
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
%% PLOT ANGULAR VELOCITY
figure
subplot(3,1,1)
plot(tout,w_B_IB(:,1))
grid on; axis on;
xlabel('[s] Time');
ylabel('\omega^b_{ib} [rad/s]');
title('Angular Velocity Relative to Inertia Frame')

subplot(3,1,2)
plot(tout,w_B_IB(:,2))
grid on; axis on;
xlabel('Time [s]');
ylabel('\omega^b_{ib} [rad/s]');

subplot(3,1,3)
plot(tout,w_B_IB(:,3))
grid on; axis on;
xlabel('Time [s]');
ylabel('\omega^b_{ib} [rad/s] ');
%% PLOT MOMENT PRODUCED BY MAGNETIC COILS
figure
subplot(3,1,1)
plot(tout,VAR.moment(:,1))
grid on; axis on;
xlabel('Time [s]');
ylabel('m_B x[Am^2]');
title('Moment Produced by Magnetic Coils')
subplot(3,1,2)
plot(tout,VAR.moment(:,2))
grid on; axis on;
xlabel('Time [s]');
ylabel('m_B y[Am^2]');

subplot(3,1,3)
plot(tout,VAR.moment(:,3))
grid on; axis on;
xlabel('Time [s]');
ylabel('m_B z[Am^2] ');

%% PLOT TORQUE PRODUCED BY MAGNETIC COILS
figure
subplot(3,1,1)
plot(tout,VAR.torque(:,1),'b')
grid on; axis on; hold on;
plot(tout,VAR.tau_g(:,1),'r')
xlabel('Time [s]');
ylabel('m_B[Nm]');
title('Torque Produced by Magnetic Coils')

subplot(3,1,2)
plot(tout,VAR.torque(:,2),'b')
grid on; axis on; hold on;
plot(tout,VAR.tau_g(:,2),'r')
xlabel('Time [s]');
ylabel('m_B[Nm]');

subplot(3,1,3)
plot(tout,VAR.torque(:,3),'b')
grid on; axis on; hold on;
plot(tout,VAR.tau_g(:,3),'r')
xlabel('Time [s]');
ylabel('m_B[Nm] ');

%% WATT
figure
plot(tout,VAR.W(:,1))
grid on; axis on;
xlabel('Time [s]');
ylabel('W');
