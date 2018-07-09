%% Spacecraft Attitude Determination Script
% Note that this code runs both the EKF and the UKF
% Created by Orlando X. Diaz
% Advisor Dr. Marcelo Romano
% Co-Advisor Dr. Hyun-wook Woo
%% Format
clear all
close all
clc
global CONST
R2D = 180/pi;
D2R = pi/180;

%% SIMULATION CONDITIONS
InitialEuler = [0,0,0];%deg
ReferenceEuler = [0 0 0];%deg

%% TOGGLE
% Toggle switches turn the labeled functions on (1) or off (0).
Tgg_toggle = 1;%
Taero_toggle = 1;%
Tsolar_toggle = 1;%
timeOn = 1;
taOn = 0;
cboOn = 0;
qbnOn = 1;
qbnmOn = 1;
rOn = 0;
hOn = 0;
e321On = 1;
wbnOn = 1;
tcOn = 0;
hsOn = 0;
wbnfOn = 1;
biasOn = 1;
biasfOn = 1;
pdOn = 1;
pnOn = 1;
qbnfOn = 1;
wbnmOn = 1;
werrOn = 1;
berrOn = 1;
qerrOn = 1;

%% CONSTANTS
CONST.mu = 398.6004418e12;%m^3/s^2
CONST.mu_moon = 4.902802953597e12;%m^3/s^2
CONST.mu_sun = 1.327122E20;%m^3/s^2
CONST.Re = 6.378137E6;%m earth radius
CONST.Rs = 1.4959787e11;%m solar radius
CONST.J2 = 1.08262668355E-3;% J2 term
CONST.J3 = -2.53265648533E-6;% J3 term
CONST.J4 = -1.61962159137E-6;% J4 term
CONST.SolarPress= 4.51e-6;%N/m^2 solar wind pressure
CONST.SOLARSEC = 806.81112382429;%TU
CONST.w_earth = -[0;0;.0000729211585530];%r/s earth rotation
CONST.Cd = 2.5;% Coefficient of Drag
CONST.Cr = .6;% Coefficient of Reflect
CONST.OmegaDot = 1.991e-7;%rad/s ascending nodeadvance for sun-synch
%% ORBITAL ELEMENTS
%Kep elements meters and radians (a,e,i,W,w,n)
h_p = 500e3;%m altitude at perigee
h_a = 500e3;%m altitude at apogee

RAAN = 0;%rad Right Ascention
w = 0;%rad argument of perigee
TAo = 0;%rad true anomaly
Rp = CONST.Re+h_p;%m radius of perigee
Ra = CONST.Re+h_a;%m radius of apogee
e = (Ra-Rp)/(Ra+Rp);%(m/m) eccentricity
a = (Ra+Rp)/2;%m semi-major axis
ho = sqrt(a*CONST.mu*(1-e^2));%mÿ2/s initial angular momentum
P = 2*pi*(a^3/CONST.mu)^.5;%sec Orbit Period
i_sunsynch = acosd((CONST.OmegaDot*(1-e^2)^2*a^(7/2))/(-3/2*sqrt(CONST.mu)*CONST.J2*CONST.Re^2));%eqn 4.47 from Curtis
i = i_sunsynch*D2R;%deg (rad) orbit inclination
i = 45/180*pi
[Ro,Vo] = sv_from_coe(CONST.mu,[ho e RAAN i w TAo]);% initial orbital state vector

%% INITIAL CONDITIONS
C_ON = DCM(1,-90*D2R)*DCM(3,(TAo+90)*D2R)*DCM(1,i);
InitialEulerO = R2eul(C_ON,'XYZ')*180/pi

w_BN_0 = [0;2*pi/P;0];%rad initial body rates
w_ON  = [0;2*pi/P;0];%rad
rand('seed',2);
randn('seed',2);
seedarw=1;
seedrrw=2;
seedst=3;
seedmag=4;

%% SENSORS PARAMETERS
% Gyro
GYRO_Bias = (3*randn(3,1))*pi/180; % + 3 deg/sec
N_ARW = (0.029)*pi/180;
K_RRW = (0.0002)*pi/180;
ARW = N_ARW^2; % angular white noise Variance
RRW = K_RRW^2/3; % bias variance
Gg = eye(3).*(-0.01+0.02*rand(3)) +...
(ones(3,3)-eye(3)).*(-0.0006+0.0012*rand(3)); %percent

% Magnetometer
sigMag = 1.25e-7;
Gm = eye(3).*(-0.02+0.04*rand(3)) +...
(ones(3,3)-eye(3)).*(-0.0028+0.0056*rand(3)); %percent

% Sun Sensor
S1 = [0 45 0]'*pi/180;
S2 = [45 0 0]'*pi/180;
SS_n1 = [1 0 0];
SS_n2 = [1 0 0];
FOV = 0.7;
sigSS = 0.1;
% J = Bessel(sigSS/2,FOV).*pi/180;

% Star Tracker
sigST = 70 /3 /60 /60*pi/180; %arcsec to rad (3sig)

% Kalman Filter
dt = 1; %sec (20 Hz) model speed
t_ekf = dt; %sec (100 Hz) ekf speed
sig(1) = sqrt(ARW); %rad/Hz^(1/2), ARW
sig(2) = sqrt(RRW); %rad/sec^(3/2), RRW
sig(3) = sigST; %rad, Star Tracker Error
sig(4) = sigSS*pi/180; %rad, Sun Sensor Error
sig(5) = sigMag; %tesla, magnetometer error
ReferenceOmega = w_ON;

qBO_0 = eul2q(InitialEuler,'xyz','xyzw')% [qBOo] = Euler_to_Quaternion(InitialEuler);
qBN_0 = eul2q(InitialEulerO,'xyz','xyzw')
[ReferenceQuaternion] = eul2q(ReferenceEuler,'xyz','xyzw'); %[ReferenceQuaternion] = Euler_to_Quaternion(ReferenceEuler);


%% Run Simulation
CONST.J = diag([0.2 0.2 0.1]);  % [kgm^2] Spacecraft Moments of Inertia
RunTime = P/4;%sec

tic
sim('satellite_attitude_kalman_ufk_ekf',RunTime);
%% POST PROCESSING
close all
R = R/CONST.Re;  % Position Vector of spacecraft

for i=1:1:length(tout)
Rx(i) = R(1,1,i); % Position Vector X of spacecraft
Ry(i) = R(2,1,i); % Position Vector Y of spacecraft
Rz(i) = R(3,1,i); % Position Vector Z of spacecraft
end
%% SIMULATION
% Figure Setting
fig = figure;
screensize = get(0,'ScreenSize');
set(fig,'Position',[0 0 screensize(4)*0.8 screensize(4)*0.8]);

% Create Earth Sphere
radius  = 1;
[x,y,z] = sphere(20); 
earth   = surf(radius*x,radius*y,radius*z);
set(earth,'facecolor','none','edgecolor',0.7*[1 1 1],'LineStyle',':'); 


grid on; axis fill;
cameratoolbar('SetMode','orbit')   

set(gca,'Position',[0 0 1 1]); % Set Position of Graph
set(gca,'CameraViewAngle',6); % Set Zoom of Graph
axis(3.5*[-1 1 -1 1 -1 1]);    % Set Limit of Axis  


 
% Earth Centered Inertial Frame
X_eci = plotVector([1 ;0 ;0], [0 0 0], 'k', 'X_eci');
Y_eci = plotVector([0 ;1 ;0], [0 0 0], 'k', 'Y_eci');
Z_eci = plotVector([0 ;0 ;1], [0 0 0], 'k', 'Z_eci');

% Satellite Position
R_sat = plotPosition(R,'b','s');
hold on;

% Satellite Body Frame
[X_sat,X_sat_lab] = plotVector(C_NB(:,:,1)'*[1 ;0 ;0], R(:,1,1), 'b', 'x_b',0.5);
[Y_sat,Y_sat_lab] = plotVector(C_NB(:,:,1)'*[0 ;1 ;0], R(:,1,1), 'b', 'y_b',0.5);
[Z_sat,Z_sat_lab] = plotVector(C_NB(:,:,1)'*[0 ;0 ;1], R(:,1,1), 'b', 'z_b',0.5);

% Orbital Frame
[X_orb,X_orb_lab] = plotVector(C_NB(:,:,1)*C_BO(:,:,1)*[1 ;0 ;0], R(:,1,1), 'g', 'x_o',0.5);
[Y_orb,Y_orb_lab] = plotVector(C_NB(:,:,1)*C_BO(:,:,1)*[0 ;1 ;0], R(:,1,1), 'g', 'y_o',0.5);
[Z_orb,Z_orb_lab] = plotVector(C_NB(:,:,1)*C_BO(:,:,1)*[0 ;0 ;1], R(:,1,1), 'g', 'z_o',0.5);

% Plot Satellite Orbit Track
plot3(Rx,Ry,Rz,'-.')
 
% UPDATE SIMULATION PLOT
d=1;
for i=1:1*d:(length(tout)-1)

rotate(earth,[0 0 1],0.005*d)
% Update Axes Label


% Update Satellite Position
updatePosition(R_sat, R(:,1,i));

% Update Orbit Frame
updateVector(X_orb, (C_NB(:,:,i)*C_BO(:,:,i))'*[1 ;0 ;0],  R(:,1,i),0.5);
updateVector(Y_orb, (C_NB(:,:,i)*C_BO(:,:,i))'*[0 ;1 ;0],  R(:,1,i),0.5);
updateVector(Z_orb, (C_NB(:,:,i)*C_BO(:,:,i))'*[0 ;0 ;1],  R(:,1,i),0.5);
set(X_orb_lab,'Position',(C_NB(:,:,i)*C_BO(:,:,i))'*[1 ;0 ;0]+R(:,1,i));
set(Y_orb_lab,'Position',(C_NB(:,:,i)*C_BO(:,:,i))'*[0 ;1 ;0]+R(:,1,i));
set(Z_orb_lab,'Position',(C_NB(:,:,i)*C_BO(:,:,i))'*[0 ;0 ;1]+R(:,1,i));

% Update Satellite Body Frame
updateVector(X_sat, C_NB(:,:,i)'*[1 ;0 ;0],  R(:,1,i),0.5);
updateVector(Y_sat, C_NB(:,:,i)'*[0 ;1 ;0],  R(:,1,i),0.5);
updateVector(Z_sat, C_NB(:,:,i)'*[0 ;0 ;1],  R(:,1,i),0.5);
set(X_sat_lab,'Position',C_NB(:,:,i)'*[1 ;0 ;0]+R(:,1,i));
set(Y_sat_lab,'Position',C_NB(:,:,i)'*[0 ;1 ;0]+R(:,1,i));
set(Z_sat_lab,'Position',C_NB(:,:,i)'*[0 ;0 ;1]+R(:,1,i));

drawnow;  
    
end

