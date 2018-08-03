close all
clear all
clc
%% DEFINING SATELLITE GEOMETRY
[R(:,1:6),A(:,1:6),N(:,1:6),dA(:,1:6)] = surfcuboid(1,1,1);
[R(:,7:8),A(:,7:8),N(:,7:8),dA(:,7:8)] = surfpanel([0;3;0],0.5,5.0);

%% SATELLITE MOTION
V = [-1;0;0];   % [m/s] Velocity Vector of Spacecraft in Body Frame
w = [0;0;-0];   %[rad/s] Angular Velocity of Spacecraft in Body Frame
rho = 1e-5;     % [kg/m^3] Density of Atmosphere

%% AERODYNAMIC TORQUE
model  = 'wertz';
tau_a1 = aero_torque(R,A,V,w,rho,model);
tau_a2 = torqaero(R,A,V,w,rho,model);
tau_a3 = torqaero(R,A,V,w,rho);

%% SOLAR TORQUE
SolarP  = 4.51e-6;           % [N/m^2] Solar Pressure
S_B     = [-1;0;0];          % [-] Sun vector 
tau_s1  = solar_torque(R,A,S_B,0,SolarP,model);
tau_s2  = torqsolar(R,A,S_B,SolarP,model);

%% GRAVITATIONAL TORQUE


%% FIGURE SETUP
fig = figure;
screensize = get(0,'ScreenSize');
set(fig,'Position',[0 0 screensize(4) screensize(4)]);   
set(gca,'Position',[0 0 1 1]); % Set Position of Graph
set(gca,'CameraViewAngle',6);  % Set Zoom of Graph

grid on; axis fill;
cameratoolbar('SetMode','orbit')
axis(3.5*[-1 1 -1 1 -1 1]);    % Set Limit of Axis  

% Earth Centered Inertial Frame
plotVector([1 ;0 ;0], [0 0 0], 'k', 'X_B',1.5);
plotVector([0 ;1 ;0], [0 0 0], 'k', 'Y_B',1.5);
plotVector([0 ;0 ;1], [0 0 0], 'k', 'Z_B',1.5);

plotVector(V, [0 0 0], 'b', 'V');
plotVector(w, [0 0 0], 'm', '\omega');

plotVector(tau_a1, [0 0 0], color('orange'), '\tau_a_e_r_o_1');
plotVector(tau_a2, [0 0 0], 'r', '\tau_a_e_r_o_2',2);
plotVector(tau_a3, [0 0 0], 'b', '\tau_a_e_r_o_3',3);

plotVector(-S_B, [0 0 0], 'y', 'SUN',3);
plotVector(tau_s1, [0 0 0], color('orange'), '\tau_s_u_n_1');
plotVector(tau_s2, [0 0 0], 'r', '\tau_s_u_n_2',2);
%% SPACECRAFT PLOT
sc_colour            = 0.8*[1 1 1];           % colour of main s/c body
sc_transparency      = 0.3;                   % transparency of s/c body
[vertices_x,vertices_y ,vertices_z]= getSurface(R,N,dA);
h_body = patch(vertices_x, vertices_y, vertices_z, 0.4*[1 1 1]);
set(h_body, 'FaceAlpha', sc_transparency)
hold on;
