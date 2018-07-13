%% Design Implementation Attitude Control Magnetic Coil Stabilization
close all
clear all
clc

%% GLOBAL CONSTANTS
global CONST

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
CONST.u_0        = pi/2;                    % [rad] Initial Sun Ascension (pi/2 - Summer, pi - Autumn, 3*pi/2 

R2D = 180/pi;
D2R = pi/180;

%% SATELLITE MOMENTS OF INERTIA
m  = 2.00;  % [kg] Satellite Mass
dx = 0.10;  % [m] Length X
dy = 0.10;  % [m] Length Y
dz = 0.10;  % [m] Length Z

Ix = (m/12)*(dy^2+dz^2); % [kg.m^2] X-axis Inertia
Iy = (m/12)*(dx^2+dz^2); % [kg.m^2] Y-axis Inertia
Iz = (m/12)*(dx^2+dy^2); % [kg.m^2] Y-axis Inertia
  
I  = diag([Ix Iy Iz]);   % [kg.m^2] Inertia 
CONST.I          = I;     % [kgm^2] Spacecraft Moments of Inertia

%% COIL PARAMETERS AND CONTROL ALLOCATION MATRIX
V_max = 5; % [V] Max voltage

Ncoil = [355;355;800];                      % Number of turns in Coil
Acoil = [0.08*0.18;0.08*0.18;0.08*0.08];    % Area of Coil 
Rcoil = [110;110;110];                      % Resistance in Coil 
i_max = [V_max/Rcoil(1);V_max/Rcoil(2);V_max/Rcoil(3)]; % 

% K Coil Matrix
K_coil = [Ncoil(1)*Acoil(1)/Rcoil(1)   0        0        ;
             0       Ncoil(2)*Acoil(2)/Rcoil(2) 0        ;
             0       0        Ncoil(3)*Acoil(3)/Rcoil(3) ];

CONST.Ncoil = Ncoil;
CONST.Acoil = Acoil;
CONST.V_max = V_max;    % [V] Voltage Maximum
CONST.K     = K_coil;   % [3x3] K coil Matrix
CONST.i_max = i_max;    % [A] Ampere Maximum

%% ORBITAL PARAMETER
h_p = 599.99e3; % [m] Altitude Perigee 
h_a = 600.01e3; % [m] Altitude Apogee

RAAN = 0;  % [rad] Initial Right Ascention - Angle on Equatorial Plane from Vernal Equinox to Ascending Node
w    = 0;  % [rad] Initial Argument of perigee - Angle on Orbital Plane from Ascending Node to Perigee
TAo  = 0;  % [deg] Initial True Anomaly - Angle on Orbital Plane from Perigee to Satellite

Rp  = CONST.Re+h_p;              % [m] radius of perigee
Ra  = CONST.Re+h_a;              % [m] radius of apogee
e   = (Ra-Rp)/(Ra+Rp);           % [m/m] eccentricity
a   = (Ra+Rp)/2;                 % [m] semi-major axis
ho  = sqrt(a*CONST.mu*(1-e^2));  % [mˆ2/s] Initial Angular momentum
P   = 2*pi*sqrt(a^3/CONST.mu);   % [sec] Orbit Period
w_O = 2*pi/P;                    % [rad/s] Orbit Angular Velocity

i = acos((CONST.OmegaDot*(1-e^2)^2*a^(7/2))/(-3/2*sqrt(CONST.mu)*CONST.J2*CONST.Re^2)); % [rad] Orbit Inclination required for Sun-synchornous (eqn 4.47 from Curtis)
i = 90.00001/180*pi;
[R_0,V_0,Q] = sv_from_coe(CONST.mu,[ho e RAAN i w TAo]);% initial orbital state vector in ECF Frame- computes the magnitude of state vector (r,v) from the classical orbital elements (coe) 

CONST.P    = P;      % [sec] Orbit Period
CONST.a    = a;      % [m] semi-major axis
CONST.w_O  = w_O;    % [rad/s] Orbit Angular Velocity
CONST.RAAN = RAAN;   % [rad] Initial Right Ascention - Angle on Equatorial Plane from Vernal Equinox to Ascending Node
CONST.i    = i;      % [rad] Orbit Inclination required for Sun-synchornous (eqn 4.47 from Curtis)
CONST.e    = e;      % [m/m] eccentricity

%% REGULATOR GAINS
K_p = 5e-08; % [-] Proportional Gain 8*w_O^2*(I(2,2)-I(3,3))
K_d = 4e-05; % [-] Differential Gain

CONST.K_d   = K_d;      % [-] Controller Gain
CONST.K_p   = K_p;      % [-] Controller Gain

%% INITIAL CONDITIONS
R_O_I   = dcm(1,-90*D2R)*DCM(3,(TAo+90)*D2R)*DCM(1,i)*DCM(3,RAAN); % [3x3] Rotation Matrix from Inertial (X axis is vernal equinox) to Orbit Frame (x is orbit direction)
Euler_O_I_0 = dcm2eul(R_O_I,'zyx'); % [rad] Initial Euler Angle transforming from Inertial Frame to Orbit Frame (ZYX means rotate X, then Y, then Z)
Euler_I_B_0 = [ -pi/4 ; 0 ; 0 ];      % [rad] Initial Euler Angle transforming from Body Frame to Inertial Frame (ZYX means rotate X, then Y, then Z)

R_I_B_0 = eul2R(Euler_I_B_0,'zyx'); % Transformation Matrix from Body to Inertial (Initial)   
R_O_I_0 = eul2R(Euler_O_I_0,'zyx'); % Transformation Matrix from Inertial to Body (Initial)
R_O_B_0 = R_O_I_0*R_I_B_0;          % Transformation Matrix from Body to Orbital (Initial)

q_I_B_0 = dcm2q(R_I_B_0,'tsf','wxyz'); % Quaternion Rotation of Body Frame from Inertial Frame
q_O_I_0 = dcm2q(R_O_I_0,'tsf','wxyz'); % Quaternion Rotation of Inertial Frame From Body Frame
q_O_B_0 = dcm2q(R_O_B_0,'tsf','wxyz'); % Quaternion Rotation of Body Frame from Orbit Frame

q_B_I_0 = [q_I_B_0(1);-q_I_B_0(2);-q_I_B_0(3);-q_I_B_0(4)];
q_B_O_0 = [q_O_B_0(1);-q_O_B_0(2);-q_O_B_0(3);-q_O_B_0(4)];

Euler_O_B_0 = dcm2eul(R_O_B_0,'zyx'); % Euler Angles from Body to Orbital (Initial)

R_E_M   = dcm(1,-11.5/180*pi); % Rotation Matrix from Magnetic to Earth (Constant)%% INITIAL ANGULAR VELOCITY

% Orbital Frame Angular Rate
w_O_OI_0 = [0;-w_O;0];             % [rad] Orbital Frame Angular Rate relative to Inertial Frame in Orbital Frame
w_B_OI_0 = R_O_B_0'*w_O_OI_0; % [rad] Orbital Frame Angular Rates relative to Inertial Frame in Body Frame

% Body Frame Angular Rate
w_B_BO_0 = [0.03;-0.02;0.05];               % [rad/s] Body Frame Angular Rates relative to Orbital Frame in Body Frame 
w_B_BI_0 = w_B_BO_0 + w_B_OI_0;   % [rad] Body Frame Angular Rate relative to Inertial Frame in Body Frame

lat_0    = 0;   % Initial Latitude
joule_0  = 0;   % Initial Work Energy

%% SIMULATION TIME
dt       = 0.1;        % [s]
tdur     = P;        % [s]
%% SIMULINK SOLVER
    
sim('satellite_control_Bdot_modified_model',tdur);
%% POST PROCESSING
close all
R = R/CONST.Re;  % Position Vector of spacecraft

for i=1:1:length(tout)
Rx(i) = R(1,1,i); % Position Vector X of spacecraft
Ry(i) = R(2,1,i); % Position Vector Y of spacecraft
Rz(i) = R(3,1,i); % Position Vector Z of spacecraft
end

close all
%% QUATERNION RELATIVE TO INERTIAL FRAME%%
% figure
% subplot(4,1,1)
% plot(tout,q_B_I(1,:),'b')
% title('Quaternion Rotation of Body to Inertial Frame')
% hold on;grid on;
% ylabel('w [-]');
% xlabel('time [s]');
% 
% subplot(4,1,2)
% plot(tout,q_B_I(2,:),'b')
% hold on;grid on;
% ylabel('x [-]');
% xlabel('time [s]');
% 
% subplot(4,1,3)
% plot(tout,q_B_I(3,:),'b')
% hold on;grid on;
% ylabel('y [-]');
% xlabel('time [s]');
% 
% subplot(4,1,4)
% plot(tout,q_B_I(4,:),'b')
% hold on;grid on;
% ylabel('z [-]');
% xlabel('time [s]');
%% QUATERNION RELATIVE TO ORBIT FRAME%%
figure
subplot(4,1,1)
plot(tout,q_B_O(1,:),'b')
title('Quaternion Rotation of Body to Orbital Frame')
hold on;grid on;
ylabel('w [-]');
xlabel('time [s]');

subplot(4,1,2)
plot(tout,q_B_O(2,:),'b')
hold on;grid on;
ylabel('x [-]');
xlabel('time [s]');

subplot(4,1,3)
plot(tout,q_B_O(3,:),'b')
hold on;grid on;
ylabel('y [-]');
xlabel('time [s]');

subplot(4,1,4)
plot(tout,q_B_O(4,:),'b')
hold on;grid on;
ylabel('z [-]');
xlabel('time [s]');

%% EULER ANGLES RELATIVE TO INERTIAL FRAME%%
% figure
% subplot(3,1,1)
% plot(tout,e_B_I(1,:)*R2D,'b');
% title('Euler Angles ZYX relative to Inertial Frame')
% hold on;grid on;
% ylabel('phi [\circ]');
% xlabel('time [s]');
% 
% subplot(3,1,2)
% plot(tout,e_B_I(2,:)*R2D,'b');
% hold on;grid on;
% ylabel('theta [\circ]');
% xlabel('time [s]');
% 
% subplot(3,1,3)
% plot(tout,e_B_I(3,:)*R2D,'b');
% hold on;grid on;
% ylabel('psi [\circ]');
% xlabel('time [s]');
%% EULER ANGLES RELATIVE TO ORBITAL FRAME%%
figure
subplot(3,1,1)
plot(tout,e_B_O(1,:)*R2D,'b');
title('Euler Angles ZYX relative to Orbital Frame')
hold on;grid on;
ylabel('phi [\circ]');
xlabel('time [s]');

subplot(3,1,2)
plot(tout,e_B_O(2,:)*R2D,'b');
hold on;grid on;
ylabel('theta [\circ]');
xlabel('time [s]');

subplot(3,1,3)
plot(tout,e_B_O(3,:)*R2D,'b');
hold on;grid on;
ylabel('psi [\circ]');
xlabel('time [s]');

%% ANGULAR VELOCITY %%
figure
subplot(3,1,1)
plot(tout,w_B_BI(1,:)*R2D,'b')
title('Angular Rates of Body to Inertial Frame')
hold on;grid on;

ylabel('w_x [\circ/s]');
xlabel('time [s]');

subplot(3,1,2)
plot(tout,w_B_BI(2,:)*R2D,'b')
hold on;grid on;
ylabel('w_y [\circ/s]');
xlabel('time [s]');

subplot(3,1,3)
plot(tout,w_B_BI(3,:)*R2D,'b')
hold on;grid on;
ylabel('w_z [\circ/s]');
xlabel('time [s]');

%% TORQUES%%
figure
subplot(3,1,1)
plot(tout,tau_m(1,:),'r');
title('Euler Angles ZYX relative to Orbital Frame')
hold on;grid on;
plot(tout,tau_g(1,:),'g');
plot(tout,tau_a(1,:),'b');
plot(tout,tau_s(1,:),'y');
ylabel('tau_x [Nm]');
xlabel('time [s]');

subplot(3,1,2)
plot(tout,tau_m(2,:),'r');
hold on;grid on;
plot(tout,tau_g(2,:),'g');
plot(tout,tau_a(1,:),'b');
plot(tout,tau_s(1,:),'y');
ylabel('tau_y [Nm]');
xlabel('time [s]');

subplot(3,1,3)
plot(tout,tau_m(3,:),'r');
hold on;grid on;
plot(tout,tau_g(3,:),'g');
plot(tout,tau_a(1,:),'b');
plot(tout,tau_s(1,:),'y');
ylabel('tau_z [Nm]');
xlabel('time [s]');

%% SIMULATION
% Figure Setting
fig = figure;
screensize = get(0,'ScreenSize');
set(fig,'Position',[0 0 screensize(4)*0.8 screensize(4)*0.8]);

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

grid on; axis fill;
cameratoolbar('SetMode','orbit')   

set(gca,'Position',[0 0 1 1]); % Set Position of Graph
set(gca,'CameraViewAngle',6); % Set Zoom of Graph
axis(3.5*[-1 1 -1 1 -1 1]);    % Set Limit of Axis  

% Satellite Position
R_sat = plotposition(R,'b','s');
hold on;

% Plot Satellite Orbit Track
plot3(Rx,Ry,Rz,'-.')

% Create Earth Sphere
radius  = 1;
[x,y,z] = sphere(20); 
earth   = surf(radius*x,radius*y,radius*z);
set(earth,'facecolor','none','edgecolor',0.7*[1 1 1],'LineStyle',':');  

% UPDATE SIMULATION PLOT
d=3;
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

drawnow;  
    
end