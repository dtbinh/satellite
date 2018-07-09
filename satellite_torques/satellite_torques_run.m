close all
clear all
clc
%% DEFINING SATELLITE GEOMETRY

dx = 1; % [m] length of x
dy = 1; % [m] length of y
dz = 1; % [m] length of z

% Main Body        
R(:,1) = [dx/2;0;0];
N(:,1) = R(:,1)/norm(R(:,1));
dA(:,1) = [dz;dy;0];
A(:,1) = dA(1,1)*dA(2,1)*N(:,1);

R(:,2) = [-dx/2;0;0];
N(:,2) = R(:,2)/norm(R(:,2));
dA(:,2) = [dz;dy;0];
A(:,2) = dA(1,2)*dA(2,2)*N(:,2);

R(:,3) = [0;dy/2;0];
N(:,3) = R(:,3)/norm(R(:,3));
dA(:,3) = [dx;dz;0];
A(:,3) = dA(1,3)*dA(2,3)*N(:,3);

R(:,4) = [0;-dy/2;0];
N(:,4) = R(:,4)/norm(R(:,4));
dA(:,4) = [dx;dz;0];
A(:,4) = dA(1,4)*dA(2,4)*N(:,4);
 
R(:,5) = [0;0;dz/2];
N(:,5) = R(:,5)/norm(R(:,5));
dA(:,5) = [dx;dy;0];
A(:,5) = dA(1,5)*dA(2,5)*N(:,5);
 
R(:,6) = [0;0;-dz/2];
N(:,6) = R(:,6)/norm(R(:,6));
dA(:,6) = [dx;dy;0];
A(:,6) = dA(1,6)*dA(2,6)*N(:,6);

% Solar Panel 1
R(:,7) = [0;-1.0;0];
surf = [1;0;0];
N(:,7) = surf/norm(surf);
dA(:,7) = [0.5;1;0];
A(:,7) = dA(1,7)*dA(2,7)*N(:,7);

R(:,8) = [0;-1.0;0];
surf = [-1;0;0];
N(:,8) = surf/norm(surf);
dA(:,8) = [0.5;1;0];
A(:,8) = dA(1,8)*dA(2,8)*N(:,8);

%% SATELLITE INSTANTENEOUS MOTION
R = R;           % [m]   Matrix of Position Vectors of each dA
A = A;           % [m^2] Matrix of Area Vector of each dA
V = [7600;0;0]; % [m/s] Velocity Vector of Spacecraft in Body Frame
w = [2;1;-2];     %[rad/s] Angular Velocity of Spacecraft in Body Frame

%% AERODYNAMIC TORQUE
rho = 1e-5;      % [kg/m^3] Density of Atmosphere
tau_a = aero_torque(R,A,V,w,rho,'hughes');

%% SOLAR TORQUE
SolarP  = 4.51e-6;                 % [N/m^2] solar wind pressure
eclipse = 0;
S_B     = [1;0;0];
tau_s   = solar_torque(R,A,S_B,eclipse,SolarP,'wertz')
tau_s   = solar_torque(R,A,S_B,eclipse,SolarP,'hughes')
%% FIGURE SETUP
fig = figure;
screensize = get(0,'ScreenSize');
set(fig,'Position',[0 0 screensize(4)*1 screensize(4)]);   
set(gca,'Position',[0 0 1 1]); % Set Position of Graph
set(gca,'CameraViewAngle',6);  % Set Zoom of Graph

grid on; axis fill;
cameratoolbar('SetMode','orbit')
axis(3.5*[-1 1 -1 1 -1 1]);    % Set Limit of Axis  

% Earth Centered Inertial Frame
X_B = plotVector([1 ;0 ;0], [0 0 0], 'k', 'X_B',1.5);
Y_B = plotVector([0 ;1 ;0], [0 0 0], 'k', 'Y_B',1.5);
Z_B = plotVector([0 ;0 ;1], [0 0 0], 'k', 'Z_B',1.5);

V   = plotVector(V, [0 0 0], 'b', 'V');
T   = plotVector(tau_a, [0 0 0], 'r', '\tau_a_e_r_o');
W   = plotVector(w, [0 0 0], 'm', '\omega');
S   = plotVector(-S_B, [0 0 0], 'y', 'SUN',3);
ST   = plotVector(tau_s, [0 0 0], [1 0.6 0], '\tau_s_u_n');
%% SPACECRAFT PLOT
sc_colour            = 0.8*[1 1 1];           % colour of main s/c body
sc_transparency      = 0.3;                   % transparency of s/c body
[vertices_x,vertices_y ,vertices_z]= getSurface(R,N,dA);
h_body = patch(vertices_x, vertices_y, vertices_z, 0.4*[1 1 1]);
set(h_body, 'FaceAlpha', sc_transparency)
hold on;
