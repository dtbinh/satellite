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
R_C_B = dcm(1,90/180*pi)*dcm(3,55/180*pi)*dcm(2,20/180*pi);
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
%% ST TRACKER ST400
R_C_B = dcm(1,-90/180*pi)*dcm(3,55/180*pi)*dcm(2,160/180*pi);
R_B_C = R_C_B';

q_C_B = dcm2q(R_C_B,'tsf','xyzw');
q_B_C = dcm2q(R_B_C,'tsf','xyzw');

q_B_C_ST400 = q_B_C;
R_B_C_ST400 = R_B_C;

XYZ_ST400   = [-0.3 0.5 -0.7];

% Plotting Euler Parameters [Sat Body to Instrument] for Checking
angle_ST400 = 2*acos(q_B_C(4))*180/pi;
e1_ST400 = q_B_C(1)/sqrt(1-q_B_C(4)^2);
e2_ST400 = q_B_C(2)/sqrt(1-q_B_C(4)^2);
e3_ST400 = q_B_C(3)/sqrt(1-q_B_C(4)^2);

fprintf('ST400 [Instrument to Body] = [%.6f %.6f %.6f %.6f]\n',q_B_C_ST400);

% StarTracker Previous

q_B_C_ST1 = [0.560991 ;0.212630 ;-0.430483 ;0.674360];
q_B_C_ST2 = [0.430483 ;0.674360 ;-0.560991 ;0.212630];

R_B_C_ST1 = q2dcm(q_B_C_ST1,'xyzw','tsf');
R_B_C_ST2 = q2dcm(q_B_C_ST2,'xyzw','tsf');
%% FLUXGATE MAGNETOMETER
fprintf('\n----------------FLUXGATE MAGNETOMETER---------------\n');

R_C_B = dcm(3,180.0001/180*pi);
R_B_C = R_C_B';

q_C_B = dcm2q(R_C_B,'tsf','xyzw');
q_B_C = dcm2q(R_B_C,'tsf','xyzw');

q_B_C_MAGM1 = q_B_C;
R_B_C_MAGM1 = R_B_C;

XYZ_MAGM1   = [0 0.75 -0.7];
% Plotting Euler Parameters [Sat Body to Instrument] for Checking
angle_MAGM1 = 2*acos(q_B_C(4))*180/pi;
e1_MAGM1 = q_B_C(1)/sqrt(1-q_B_C(4)^2);
e2_MAGM1 = q_B_C(2)/sqrt(1-q_B_C(4)^2);
e3_MAGM1 = q_B_C(3)/sqrt(1-q_B_C(4)^2);

fprintf('Quaternion [Instrument to Body] = [%.6f %.6f %.6f %.6f]\n',q_B_C_MAGM1);

%% LSM303D Accelerometer and Magnetometer
fprintf('\n----------------LSM303D Accelerometer and Magnetometer ---------------\n');

R_B_C_LSM303D = dcm(3,179.9999/180*pi)*dcm(1,90/180*pi);
q_B_C_LSM303D = dcm2q(R_B_C_LSM303D,'tsf','xyzw');

XYZ_LSM303D   = [0 0.0 0.3];

fprintf('Quaternion [Instrument to Body] = [%.6f %.6f %.6f %.6f]\n',q_B_C_LSM303D);

%% L3GD20H MEMS GYRO
fprintf('\n---------------L3GD20H MEMS GYRO---------------\n');

R_B_C_L3GD20H = dcm(1,-90/180*pi);
q_B_C_L3GD20H = dcm2q(R_B_C_L3GD20H,'tsf','xyzw');

XYZ_L3GD20H  = [0 0.0 -0.3];

fprintf('Quaternion [Instrument to Body] = [%.6f %.6f %.6f %.6f]\n',q_B_C_L3GD20H);

%% A3G4250D MEMS GYRO
fprintf('\n---------------A3G4250D MEMS GYRO---------------\n');

R_B_C_A3G4250D = dcm(2,90/180*pi)*dcm(1,-90/180*pi);
q_B_C_A3G4250D = dcm2q(R_B_C_A3G4250D,'tsf','xyzw');

XYZ_A3G4250D  = [0.1 0.0 0.0];

fprintf('Quaternion [Instrument to Body] = [%.6f %.6f %.6f %.6f]\n',q_B_C_A3G4250D);
%% REACTION WHEELS
fprintf('\n----------REACTION WHEELS ASSEMBLY---------------\n');

R_B_C_RWA(:,:,1) = dcm(1,90/180*pi)*dcm(3,179.99/180*pi);
q_B_C_RWA(:,1) = dcm2q(R_B_C_RWA(:,:,1),'tsf','xyzw');
XYZ_RWA(:,1) = [0;1;0.9];

R_B_C_RWA(:,:,2) = dcm(3,-90/180*pi)*dcm(1,-90/180*pi);
q_B_C_RWA(:,2) = dcm2q(R_B_C_RWA(:,:,2),'tsf','xyzw');
XYZ_RWA(:,2) = [0;1;-0.1];

R_B_C_RWA(:,:,3) = dcm(3,179.99/180*pi)*dcm(2,-90/180*pi);
q_B_C_RWA(:,3) = dcm2q(R_B_C_RWA(:,:,3),'tsf','xyzw');
XYZ_RWA(:,3) = [1;1;0];

for i=1:length(XYZ_RWA)
fprintf('Reaction Wheel %d [Instrument to Body] = [%.6f %.6f %.6f %.6f]\n',i,q_B_C_RWA(:,i));
end
for i=1:length(XYZ_RWA)
fprintf('Fibre Optical Gyro %d [Instrument to Body] = [%.6f %.6f %.6f %.6f]\n',i,q_B_C_RWA(:,i));
end
%% MAGNETORQUER
fprintf('\n----------------MAGNETORQUER----------------\n');

R_B_C_MTQ(:,:,1) = dcm(3,179.99/180*pi)*dcm(1,90/180*pi);
q_B_C_MTQ(:,1) = dcm2q(R_B_C_MTQ(:,:,1),'tsf','xyzw');
XYZ_MTQ(:,1) = [0;-1;-1];

R_B_C_MTQ(:,:,2) = dcm(3,90/180*pi)*dcm(1,90/180*pi);
q_B_C_MTQ(:,2) = dcm2q(R_B_C_MTQ(:,:,2),'tsf','xyzw');
XYZ_MTQ(:,2) = [-1;-1;0];

R_B_C_MTQ(:,:,3) = dcm(3,179.99/180*pi)*dcm(2,-90/180*pi);
q_B_C_MTQ(:,3) = dcm2q(R_B_C_MTQ(:,:,3),'tsf','xyzw');
XYZ_MTQ(:,3) = [1;-1;0];

for i=1:length(XYZ_MTQ)
fprintf('Magnetorquer %d [Instrument to Body] = [%.6f %.6f %.6f %.6f]\n',i,q_B_C_MTQ(:,i));
end

%% SUN SENSORS
fprintf('\n----------------SUN SENSORS---------------\n');

% 1
R_B_C_SS(:,:,1) = dcm(1,-90/180*pi)*dcm(3,45/180*pi);
q_B_C_SS(:,1) = dcm2q(R_B_C_SS(:,:,1),'tsf','xyzw');
XYZ_SS(:,1) = [-0.9;-1;-0.9];

% 2
R_B_C_SS(:,:,2) = dcm(2,90/180*pi)*dcm(3,-90/180*pi);
q_B_C_SS(:,2) = dcm2q(R_B_C_SS(:,:,2),'tsf','xyzw');
XYZ_SS(:,2) = [-1;-0.9;-0.9];
% 3
R_B_C_SS(:,:,3) = dcm(3,90/180*pi)*dcm(1,180.001/180*pi);
q_B_C_SS(:,3) = dcm2q(R_B_C_SS(:,:,3),'tsf','xyzw');
XYZ_SS(:,3) = [-0.9;-0.9;-1];
% 4
R_B_C_SS(:,:,4) = dcm(2,-90/180*pi);
q_B_C_SS(:,4) = dcm2q(R_B_C_SS(:,:,4),'tsf','xyzw');
XYZ_SS(:,4) = [1;0.5;0.9];
% 5
R_B_C_SS(:,:,5) = dcm(3,179.999/180*pi);
q_B_C_SS(:,5) = dcm2q(R_B_C_SS(:,:,5),'tsf','xyzw');
XYZ_SS(:,5) = [0.9;0.9;1];
% 6
R_B_C_SS(:,:,6) = dcm(3,90/180*pi)*dcm(2,90/180*pi);
q_B_C_SS(:,6) = dcm2q(R_B_C_SS(:,:,6),'tsf','xyzw');
XYZ_SS(:,6) = [0.7;1;0.9];

for i=1:length(XYZ_SS)
fprintf('Sun Sensor %d [Instrument to Body] = [%.6f %.6f %.6f %.6f]\n',i,q_B_C_SS(:,i));
end


%% SOLAR PANELS
fprintf('\n----------------SOLAR PANELS---------------\n');
R_B_C_SP(:,:,1) = dcm(1,90/180*pi)*dcm(3,179.9999/180*pi);
q_B_C_SP(:,1)   = dcm2q(R_B_C_SP(:,:,1),'tsf','xyzw');
XYZ_SP(:,1)     = [-1;0.8;0];


R_B_C_SP(:,:,2) = dcm(1,90/180*pi);
q_B_C_SP(:,2)   = dcm2q(R_B_C_SP(:,:,2),'tsf','xyzw');
XYZ_SP(:,2)     = [1;0.8;0];

R_B_C_SP(:,:,3) = dcm(3,90/180*pi)*dcm(2,-90/180*pi);
q_B_C_SP(:,3)   = dcm2q(R_B_C_SP(:,:,3),'tsf','xyzw');
XYZ_SP(:,3)     = [0;0.8;-1];

R_B_C_SP(:,:,4) = dcm(3,90/180*pi)*dcm(1,-90/180*pi);
q_B_C_SP(:,4)   = dcm2q(R_B_C_SP(:,:,4),'tsf','xyzw');
XYZ_SP(:,4)     = [-0.75;-1;0];


R_B_C_SP(:,:,5) = dcm(3,90/180*pi)*dcm(1,-90/180*pi);
q_B_C_SP(:,5)   = dcm2q(R_B_C_SP(:,:,5),'tsf','xyzw');
XYZ_SP(:,5)     = [0;-1;0];


R_B_C_SP(:,:,6) = dcm(3,90/180*pi)*dcm(1,-90/180*pi);
q_B_C_SP(:,6)   = dcm2q(R_B_C_SP(:,:,6),'tsf','xyzw');
XYZ_SP(:,6)     = [0.75;-1;0];

for i=1:length(XYZ_SP)
fprintf('Solar Panels %d [Instrument to Body] = [%.6f %.6f %.6f %.6f]\n',i,q_B_C_SP(:,i));
end

% Optimal Sub Pointing Vector
S_P = [0.3015;-0.9045;-0.3015];
Z   = [0;0;1];
Y   = [0;1;0];
X   = [1;0;0];
% angle_z = acos(dot(S_P,Z)/norm(S_P)/norm(Z))/pi*180
% angle_y = acos(dot(S_P,Y)/norm(S_P)/norm(Y))/pi*180
% angle_x = acos(dot(S_P,X)/norm(S_P)/norm(X))/pi*180

% Effective Area of each Solar Panels in Inertial Frame
% Vector and Magnitude
SP_Area  = [2 2 1 2 2 1];
Area_eff = 6.6332;
for i=1:length(XYZ_SP)
   A_SP(:,i) = R_B_C_SP(:,:,i)*[1;0;0]*SP_Area(i);    
end

% lagari_optimum_sun;

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

% Payload
sc_cyl2_radius       = 0.5;     % radius in XY plane
sc_cyl2_height       = 0.8;   % along Z axis direction
sc_cyl2_x_offset     = 0.4;       % baseplate offset w.r.t XY plane
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

% ST400 Frame
X_st400 = plotvector(R_B_C_ST400*[1 ;0 ;0],XYZ_ST400, 'r', 'X_S_T_4_0_0',1);
Y_st400 = plotvector(R_B_C_ST400*[0 ;1 ;0],XYZ_ST400, 'g', 'Y_S_T_4_0_0',0.5);
Z_st400 = plotvector(R_B_C_ST400*[0 ;0 ;1],XYZ_ST400, 'b', 'Z_S_T_4_0_0',0.5);

% ST1 Frame
% plotvector(R_B_C_ST1*[1 ;0 ;0],XYZ_ST200, 'r', 'X_S_T_1',1);
% plotvector(R_B_C_ST1*[0 ;1 ;0],XYZ_ST200, 'g', 'Y_S_T_1',0.5);
% plotvector(R_B_C_ST1*[0 ;0 ;1],XYZ_ST200, 'b', 'Z_S_T_1',0.5);
% 
% % ST2 Frame
% plotvector(R_B_C_ST2*[1 ;0 ;0],XYZ_ST400, 'r', 'X_S_T_2',1);
% plotvector(R_B_C_ST2*[0 ;1 ;0],XYZ_ST400, 'g', 'Y_S_T_2',0.5);
% plotvector(R_B_C_ST2*[0 ;0 ;1],XYZ_ST400, 'b', 'Z_S_T_2',0.5);


% MAGM1 Frame
X_MAGM1 = plotvector(R_B_C_MAGM1*[1 ;0 ;0],XYZ_MAGM1, 'r', 'X_M_A_G_M_1',0.5);
Y_MAGM1 = plotvector(R_B_C_MAGM1*[0 ;1 ;0],XYZ_MAGM1, 'g', 'Y_M_A_G_M_1',0.5);
Z_MAGM1 = plotvector(R_B_C_MAGM1*[0 ;0 ;1],XYZ_MAGM1, 'b', 'Z_M_A_G_M_1',0.5);
% LSM303D Frame
X_LSM303D = plotvector(R_B_C_LSM303D*[1 ;0 ;0],XYZ_LSM303D, 'r', 'X_L_S_M_3_0_3_D',0.5);
Y_LSM303D = plotvector(R_B_C_LSM303D*[0 ;1 ;0],XYZ_LSM303D, 'g', 'Y_L_S_M_3_0_3_D',0.5);
Z_LSM303D = plotvector(R_B_C_LSM303D*[0 ;0 ;1],XYZ_LSM303D, 'b', 'Z_L_S_M_3_0_3_D',0.5);
% L3GD20H Frame
X_L3GD20H = plotvector(R_B_C_L3GD20H*[1 ;0 ;0],XYZ_L3GD20H, 'r', 'X_L_3_G_D_2_0_H',0.5);
Y_L3GD20H = plotvector(R_B_C_L3GD20H*[0 ;1 ;0],XYZ_L3GD20H, 'g', 'Y_L_3_G_D_2_0_H',0.5);
Z_L3GD20H = plotvector(R_B_C_L3GD20H*[0 ;0 ;1],XYZ_L3GD20H, 'b', 'Z_L_3_G_D_2_0_H',0.5);
% MAGM1 Frame
X_A3G4250D = plotvector(R_B_C_A3G4250D*[1 ;0 ;0],XYZ_A3G4250D, 'r', 'X_A_3_G_4_2_5_0_D',0.5);
Y_A3G4250D = plotvector(R_B_C_A3G4250D*[0 ;1 ;0],XYZ_A3G4250D, 'g', 'Y_A_3_G_4_2_5_0_D',0.5);
Z_A3G4250D = plotvector(R_B_C_A3G4250D*[0 ;0 ;1],XYZ_A3G4250D, 'b', 'Z_A_3_G_4_2_5_0_D',0.5);

% Sun Sensors Frames
for i=1:length(XYZ_SS)
plotvector(R_B_C_SS(:,:,i)*[1 ;0 ;0],XYZ_SS(:,i), [0.9 0.55 0.25], strcat('X_S_S_',num2str(i)),0.25);
plotvector(R_B_C_SS(:,:,i)*[0 ;1 ;0],XYZ_SS(:,i), [0.9 0.63 0.25], strcat('Y_S_S_',num2str(i)),0.25);
plotvector(R_B_C_SS(:,:,i)*[0 ;0 ;1],XYZ_SS(:,i), [0.9 0.80 0.25], strcat('Z_S_S_',num2str(i)),0.25);
    
end

% Reaction Wheel Frame
for i=1:length(XYZ_RWA)
plotvector(R_B_C_RWA(:,:,i)*[1 ;0 ;0],XYZ_RWA(:,i), [0.25 0.50 0.9], strcat('X_R_W_A_',num2str(i)),0.25);
plotvector(R_B_C_RWA(:,:,i)*[0 ;1 ;0],XYZ_RWA(:,i), [0.25 0.35 0.9], strcat('Y_R_W_A_',num2str(i)),0.25);
plotvector(R_B_C_RWA(:,:,i)*[0 ;0 ;1],XYZ_RWA(:,i), [0.25 0.20 0.9], strcat('Z_R_W_A_',num2str(i)),0.25);
    
end

% Magnetorquer
for i=1:length(XYZ_MTQ)
plotvector(R_B_C_MTQ(:,:,i)*[1 ;0 ;0],XYZ_MTQ(:,i), [1 0.17 0.75], strcat('X_M_T_Q_',num2str(i)),0.50);
plotvector(R_B_C_MTQ(:,:,i)*[0 ;1 ;0],XYZ_MTQ(:,i), [1 0.17 0.65], strcat('Y_M_T_Q_',num2str(i)),0.25);
plotvector(R_B_C_MTQ(:,:,i)*[0 ;0 ;1],XYZ_MTQ(:,i), [1 0.17 0.55], strcat('Z_M_T_Q_',num2str(i)),0.25);
    
end

% Solar Panel Frames
for i=1:length(XYZ_SP)
plotvector(R_B_C_SP(:,:,i)*[1 ;0 ;0],XYZ_SP(:,i), [0.8 0.8 0], strcat('X_S_P_',num2str(i)),0.50);
plotvector(R_B_C_SP(:,:,i)*[0 ;1 ;0],XYZ_SP(:,i), [0.7 0.7 0], strcat('Y_S_P_',num2str(i)),0.25);
plotvector(R_B_C_SP(:,:,i)*[0 ;0 ;1],XYZ_SP(:,i), [0.6 0.6 0], strcat('Z_S_P_',num2str(i)),0.25);
    
end
% plotvector(V_S, [0;0;0], 'r', 'Optimal Sun Pointing',3);
% ST200/ST400 Rotation Vector
% plotvector([e1_ST200 ;e2_ST200 ;e3_ST200], [0;0;0], 'k', 'e_S_T_2_0_0',1);
% plotvector([e1_ST400 ;e2_ST400 ;e3_ST400], [0;0;0], 'k', 'e_S_T_4_0_0',1);

