close all
clear all
clc
%% FORMAT SETTING
format

%% TEST
fprintf('\n-------------------ORIENTATION TEST-------------------\n');

R_C_B = dcm(1,90/180*pi)*dcm(3,0/180*pi)*dcm(2,0/180*pi);
q_C_B = dcm2q(R_C_B,'tsf','xyzw');

e_C_B = [1;0;0];    % Rotation Axis
t_C_B = 90/180*pi;  % Rotation Angle

q_C_B = [   e_C_B(1)*sin(t_C_B/2);
            e_C_B(2)*sin(t_C_B/2);
            e_C_B(3)*sin(t_C_B/2);
                  cos(t_C_B/2)  ];

R_C_B = q2dcm(q_C_B,'xyzw','tsf');

R_B_C_TEST = R_C_B';
fprintf('Quaternion [Instrument to Body] = [%.6f %.6f %.6f %.6f]\n',q_C_B);

%% ST TRACKER ST200
fprintf('\n----------------STAR TRACKERS---------------------\n');
R_C_B = dcm(2,30/180*pi)*dcm(3,110/180*pi);
R_B_C = R_C_B';

q_C_B = dcm2q(R_C_B,'tsf','xyzw');
q_B_C = dcm2q(R_B_C,'tsf','xyzw');

q_B_C_ST200 = q_B_C;
R_B_C_ST200 = R_B_C;

XYZ_ST200   = [0.3 0.5 -0.7];

% Plotting Euler Parameters [Sat Body to Instrument] for Checking
angle_ST200 = 2*acos(q_B_C(4))*180/pi;
e1_ST200 = q_B_C(1)/sqrt(1-q_B_C(4)^2);
e2_ST200 = q_B_C(2)/sqrt(1-q_B_C(4)^2);
e3_ST200 = q_B_C(3)/sqrt(1-q_B_C(4)^2);

fprintf('ST200 [Instrument to Body] = [%.6f %.6f %.6f %.6f]\n',q_B_C_ST200);

%% PLOT
%  Figure Setting
fig = figure;
screensize = get(0,'ScreenSize');
set(fig,'Position',[0 0 screensize(3)*0.5 screensize(4)]);
grid on; axis fill;
cameratoolbar('SetMode','orbit')   
cameratoolbar('Show')
cameratoolbar('SetCoordSys','none')
view([1,-1,1])
set(gca,'Position',[0 0 1 1]); % Set Position of Graph
set(gca,'CameraViewAngle',4); % Set Zoom of Graph
axis(3.5*[-1 1 -1 1 -1 1]);    % Set Limit of Axis

%% SPACECRAFT PLOT
sc_colour            = 0.8*[1 1 1];           % colour of main s/c body
sc_transparency      = 0.3;                   % transparency of s/c body
sc_body_width        =[2 2 2];

%% DEFINING SATELLITE GEOMETRY

[R,N,dA,A] = satbody(sc_body_width(1),sc_body_width(2),sc_body_width(3));
[vertices_x,vertices_y ,vertices_z]= getsurface(R,N,dA);
h_body = patch(vertices_x, vertices_y, vertices_z, 0.8*[1 1 1]);
set(h_body, 'FaceAlpha', sc_transparency)
hold on;

XYZ_ORIGIN  = [0 0 0];

% Payload 1
sc_cyl1_radius       = 0.3;     % radius in XY plane
sc_cyl1_height       = 0.2;   % along Z axis direction
sc_cyl1_x_offset     = 0.4;       % baseplate offset w.r.t XY plane
sc_cyl1_y_offset     = -0.4;       % baseplate offset w.r.t XY plane
sc_cyl1_z_offset     = 1;       % baseplate offset w.r.t XY plane
sc_cyl1_colour       = 'g';
sc_cyl1_transparency = sc_transparency;
sc_cyl1_mesh_points  = 10;

[x_cyl1,y_cyl1,z_cyl1] = cylinder(sc_cyl1_radius*[1 1], sc_cyl1_mesh_points);
h_cyl1                 = surf(x_cyl1+sc_cyl1_x_offset, y_cyl1+sc_cyl1_y_offset, sc_cyl1_height*z_cyl1 + sc_cyl1_z_offset, ...
                              'FaceColor', sc_cyl1_colour, 'FaceAlpha', sc_cyl1_transparency, 'EdgeAlpha', sc_cyl1_transparency);
                          
% Payload 2
sc_cyl2_radius       = 0.3;     % radius in XY plane
sc_cyl2_height       = 0.2;   % along Z axis direction
sc_cyl2_x_offset     = -0.4;       % baseplate offset w.r.t XY plane
sc_cyl2_y_offset     = -0.4;       % baseplate offset w.r.t XY plane
sc_cyl2_z_offset     = 1;       % baseplate offset w.r.t XY plane
sc_cyl2_colour       = 'r';
sc_cyl2_transparency = sc_transparency;
sc_cyl2_mesh_points  = 10;

[x_cyl2,y_cyl2,z_cyl2] = cylinder(sc_cyl2_radius*[1 1], sc_cyl2_mesh_points);
h_cyl2                 = surf(x_cyl2+sc_cyl2_x_offset, y_cyl2+sc_cyl2_y_offset, sc_cyl2_height*z_cyl2 + sc_cyl2_z_offset, ...
                              'FaceColor', sc_cyl2_colour, 'FaceAlpha', sc_cyl2_transparency, 'EdgeAlpha', sc_cyl2_transparency);

% Spacecraft Body Frame
X_sat = plotvector([1 ;0 ;0], XYZ_ORIGIN, 'r', 'X_s_a_t');
Y_sat = plotvector([0 ;1 ;0], XYZ_ORIGIN, 'g', 'Y_s_a_t');
Z_sat = plotvector([0 ;0 ;1], XYZ_ORIGIN, 'b', 'Z_s_a_t',2);

% TEST Frame
% X_st = plotvector(R_B_C_TEST*[1 ;0 ;0], [0 0 0], 'r', 'X_T_E_S_T',0.7);
% Y_st = plotvector(R_B_C_TEST*[0 ;1 ;0], [0 0 0], 'g', 'Y_T_E_S_T',0.7);
% Z_st = plotvector(R_B_C_TEST*[0 ;0 ;1], [0 0 0], 'b', 'Z_T_E_S_T',0.7);

% ST200 Frame
X_st200 = plotvector(R_B_C_ST200*[1 ;0 ;0],XYZ_ST200, 'r', 'X_S_T_2_0_0',1);
Y_st200 = plotvector(R_B_C_ST200*[0 ;1 ;0],XYZ_ST200, 'g', 'Y_S_T_2_0_0',0.5);
Z_st200 = plotvector(R_B_C_ST200*[0 ;0 ;1],XYZ_ST200, 'b', 'Z_S_T_2_0_0',0.5);


