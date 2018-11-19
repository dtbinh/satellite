close all
clear all
clc
%% FORMAT SETTING
format

%% SET UP FIGURE
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
set(gca,'CameraViewAngle',8); % Set Zoom of Graph
axis(4.5*[-1 1 -1 1 -1 1]);    % Set Limit of Axis

%% SPACECRAFT GEOMETRY PARAMETERS
fprintf('\n------------------OP-SATS---------------------\n');
sc_colour         = 0.8*[1 1 1];           % colour of main s/c body
sc_transparency   = 0.3;                   % transparency of s/c body
sc_body_width     = [2 2 6];               % satellite sizing
X_Y_Z_ORIGIN       = [-1 1 3];

[R,N,dA,A]          = satbody(sc_body_width(1),sc_body_width(2),sc_body_width(3));
[x_sat,y_sat,z_sat] = getsurface(R,N,dA);
h_body              = patch(x_sat, y_sat, z_sat, 0.8*[1 1 1]);
set(h_body, 'FaceAlpha', sc_transparency)

% PLOT SATELLITE FRAME
X_sat = plotvector([1 ;0 ;0], X_Y_Z_ORIGIN, 'r', 'X_s_a_t');
Y_sat = plotvector([0 ;1 ;0], X_Y_Z_ORIGIN, 'g', 'Y_s_a_t');
Z_sat = plotvector([0 ;0 ;1], X_Y_Z_ORIGIN, 'b', 'Z_s_a_t',2);
hold on;

%% iADCS
fprintf('\n-----------------iADCS---------------------\n');
R_B_iADCS = dcm(3,90/180*pi)*dcm(2,180/180*pi); % Rotation Matrix from Component Frame to Satellite Frame
q_B_iADCS = dcm2q(R_B_iADCS,'tsf','xyzw');
X_Y_iADCS = [0.0 0.0 -0.5];

fprintf('iADCS [iAq2dcmDCS to Body] = [%.12f, %.12f, %.12f, %.12f]\n',q_B_iADCS);

X_iADCS = plotvector(R_B_iADCS*[1 ;0 ;0], X_Y_iADCS, 'r', 'X_i_a_d_c_s');
Y_iADCS = plotvector(R_B_iADCS*[0 ;1 ;0], X_Y_iADCS, 'g', 'Y_i_a_d_c_s');
Z_iADCS = plotvector(R_B_iADCS*[0 ;0 ;1], X_Y_iADCS, 'b', 'Z_i_a_d_c_s');

%% ST TRACKER ST200
fprintf('\n----------------STAR TRACKERS---------------------\n');

% iADCS
R_iADCS_C_ST200 = dcm(3,-(90+30.15)/180*pi);
q_iADCS_C_ST200 = dcm2q(R_iADCS_C_ST200,'tsf','xyzw');

% OPSSAT
R_B_C_ST200   = R_B_iADCS*R_iADCS_C_ST200;
q_B_C_ST200   = dcm2q(R_B_C_ST200,'tsf','xyzw');

X_Y_Z_ST200 = [0.5 0.5 -0.2];

% Serial
fprintf('ST200 [Instrument to iADCS] = [%.12f, %.12f, %.12f, %.12f]\n',q_iADCS_C_ST200);
fprintf('ST200 [Instrument to Body] = [%.12f, %.12f, %.12f, %.12f]\n',q_B_C_ST200);

XYZ_ORIGIN  = [0 0 0];

% Plot
X_st200 = plotvector(R_B_C_ST200*[1 ;0 ;0],X_Y_Z_ST200, 'r', 'X_S_T_2_0_0',1);
Y_st200 = plotvector(R_B_C_ST200*[0 ;1 ;0],X_Y_Z_ST200, 'g', 'Y_S_T_2_0_0',0.5);
Z_st200 = plotvector(R_B_C_ST200*[0 ;0 ;1],X_Y_Z_ST200, 'b', 'Z_S_T_2_0_0',0.5);

%% REACTION WHEELS
fprintf('\n----------REACTION WHEELS ASSEMBLY---------------\n');

% Reaction Wheel X
R_iADCS_C_RWA(:,:,1) = dcm(1,0/180*pi);
q_iADCS_C_RWA(:,1)   = dcm2q(R_iADCS_C_RWA(:,:,1),'tsf','xyzw');

R_B_C_RWA(:,:,1)     = R_B_iADCS*R_iADCS_C_RWA(:,:,1);
q_B_C_RWA(:,1)       = dcm2q(R_B_C_RWA(:,:,1),'tsf','xyzw');

X_Y_Z_RWA(:,1)        = [0.75;0.75;-0.2];

% Reaction Wheel Y
R_iADCS_C_RWA(:,:,2) = dcm(3,-90/180*pi);
q_iADCS_C_RWA(:,2)   = dcm2q(R_iADCS_C_RWA(:,:,2),'tsf','xyzw');

R_B_C_RWA(:,:,2)     = R_B_iADCS*R_iADCS_C_RWA(:,:,2);
q_B_C_RWA(:,2)       = dcm2q(R_B_C_RWA(:,:,2),'tsf','xyzw');

X_Y_Z_RWA(:,2)       = [-1;-0.5;-0.2];

% Reaction Wheel Z
R_iADCS_C_RWA(:,:,3) = dcm(2,90/180*pi);
q_iADCS_C_RWA(:,3)   = dcm2q(R_iADCS_C_RWA(:,:,3),'tsf','xyzw');

R_B_C_RWA(:,:,3)     = R_B_iADCS*R_iADCS_C_RWA(:,:,3);
q_B_C_RWA(:,3)       = dcm2q(R_B_C_RWA(:,:,3),'tsf','xyzw');

X_Y_Z_RWA(:,3)       = [-1;0.5;-0.2];

% Report
for i=1:length(X_Y_Z_RWA)
fprintf('Reaction Wheel %d [Instrument to iADCS] = [%.12f, %.12f, %.12f, %.12f]\n',i,q_iADCS_C_RWA(:,i));
fprintf('Reaction Wheel %d [Instrument to Body] = [%.12f, %.12f, %.12f, %.12f]\n',i,q_B_C_RWA(:,i));
fprintf('Rotation Matrix Reaction Wheel %d [Instrument to Body] = \n %.12f, %.12f, %.12f, \n %.12f, %.12f, %.12f, \n %.12f, %.12f, %.12f, \n\n',i,R_B_C_RWA(:,:,i));

end

% Plot
for i=1:length(X_Y_Z_RWA)
plotvector(R_B_C_RWA(:,:,i)*[1 ;0 ;0],X_Y_Z_RWA(:,i), [0.25 0.50 0.9], strcat('X_R_W_A_',num2str(i)),0.5);
plotvector(R_B_C_RWA(:,:,i)*[0 ;1 ;0],X_Y_Z_RWA(:,i), [0.25 0.35 0.9], strcat('Y_R_W_A_',num2str(i)),0.25);
plotvector(R_B_C_RWA(:,:,i)*[0 ;0 ;1],X_Y_Z_RWA(:,i), [0.25 0.20 0.9], strcat('Z_R_W_A_',num2str(i)),0.25);
    
end

%% MAGNETORQUER
fprintf('\n----------MAGNETORQUER---------------\n');
% MAGNETORQUER X
R_iADCS_C_MTQ(:,:,1) = dcm(1,0/180*pi);
q_iADCS_C_MTQ(:,1)   = dcm2q(R_iADCS_C_MTQ(:,:,1),'tsf','xyzw');

R_B_C_MTQ(:,:,1)     = R_B_iADCS*R_iADCS_C_MTQ(:,:,1);
q_B_C_MTQ(:,1)       = dcm2q(R_B_C_MTQ(:,:,1),'tsf','xyzw');

X_Y_Z_MTQ(:,1)        = [-0.9;0;-0.4];

% MAGNETORQUER Y
R_iADCS_C_MTQ(:,:,2) = dcm(3,-90/180*pi);
q_iADCS_C_MTQ(:,2)   = dcm2q(R_iADCS_C_MTQ(:,:,2),'tsf','xyzw');

R_B_C_MTQ(:,:,2)     = R_B_iADCS*R_iADCS_C_MTQ(:,:,2);
q_B_C_MTQ(:,2)       = dcm2q(R_B_C_MTQ(:,:,2),'tsf','xyzw');

X_Y_Z_MTQ(:,2)       = [0.0;1;-0.4];

% MAGNETORQUER Z
R_iADCS_C_MTQ(:,:,3) = dcm(2,90/180*pi);
q_iADCS_C_MTQ(:,3)   = dcm2q(R_iADCS_C_MTQ(:,:,3),'tsf','xyzw');

R_B_C_MTQ(:,:,3)     = R_B_iADCS*R_iADCS_C_MTQ(:,:,3);
q_B_C_MTQ(:,3)       = dcm2q(R_B_C_MTQ(:,:,3),'tsf','xyzw');

X_Y_Z_MTQ(:,3)       = [0.8;-0.8;-0.0];

% Report
for i=1:length(X_Y_Z_MTQ)
fprintf('Magnetic Torquer %d [Instrument to iADCS] = [%.12f, %.12f, %.12f, %.12f]\n',i,q_iADCS_C_MTQ(:,i));
fprintf('Magnetic Torquer %d [Instrument to Body] = [%.12f, %.12f, %.12f, %.12f]\n',i,q_B_C_MTQ(:,i));
end

% Plot
for i=1:length(X_Y_Z_MTQ)
plotvector(R_B_C_MTQ(:,:,i)*[1 ;0 ;0],X_Y_Z_MTQ(:,i), [1 0.17 0.75], strcat('X_M_T_Q_',num2str(i)),0.5);
plotvector(R_B_C_MTQ(:,:,i)*[0 ;1 ;0],X_Y_Z_MTQ(:,i), [1 0.17 0.65], strcat('Y_M_T_Q_',num2str(i)),0.2);
plotvector(R_B_C_MTQ(:,:,i)*[0 ;0 ;1],X_Y_Z_MTQ(:,i), [1 0.17 0.55], strcat('Z_M_T_Q_',num2str(i)),0.2);
    
end
%% L3GD20 GYRO (0)
fprintf('\n-----------L3GD20 GYRO---------------\n');

R_iADCS_C_L3GD20 = dcm(3,-90/180*pi);
q_iADCS_C_L3GD20  = dcm2q(R_iADCS_C_L3GD20(:,:,1),'tsf','xyzw');

R_B_C_L3GD20     = R_B_iADCS*R_iADCS_C_L3GD20(:,:,1);
q_B_C_L3GD20      = dcm2q(R_B_C_L3GD20(:,:,1),'tsf','xyzw');
fprintf('L3GD20 Gyro [Instrument to iADCS] = [%.12f, %.12f, %.12f, %.12f]\n',q_iADCS_C_L3GD20);
fprintf('L3GD20 Gyro [Instrument to Body] = [%.12f, %.12f, %.12f, %.12f]\n',q_B_C_L3GD20);
X_Y_Z_L3GD20        = [0.0;-2;-0.2];
% Plot
plotvector(R_B_C_L3GD20*[1 ;0 ;0],X_Y_Z_L3GD20, [0.25 0.50 0.9], strcat('X_L_3_G_D_2_0',0.5));
plotvector(R_B_C_L3GD20*[0 ;1 ;0],X_Y_Z_L3GD20, [0.25 0.35 0.9], strcat('Y_L_3_G_D_2_0',0.25));
plotvector(R_B_C_L3GD20*[0 ;0 ;1],X_Y_Z_L3GD20, [0.25 0.20 0.9], strcat('Z_L_3_G_D_2_0',0.25)); 



%% A3G4250DSPI GYRO (1)
fprintf('\n-----------A3G4250DSPI GYRO---------------\n');

R_iADCS_C_A3G4250DSPI = dcm(3,90/180*pi);
q_iADCS_C_A3G4250DSPI   = dcm2q(R_iADCS_C_A3G4250DSPI(:,:,1),'tsf','xyzw');

R_B_C_A3G4250DSPI     = R_B_iADCS*R_iADCS_C_A3G4250DSPI(:,:,1);
q_B_C_A3G4250DSPI       = dcm2q(R_B_C_A3G4250DSPI(:,:,1),'tsf','xyzw');
fprintf('A3G4250DSPI Gyro [Instrument to iADCS] = [%.12f, %.12f, %.12f, %.12f]\n',q_iADCS_C_A3G4250DSPI);
fprintf('A3G4250DSPI Gyro [Instrument to Body] = [%.12f, %.12f, %.12f, %.12f]\n',q_B_C_A3G4250DSPI);
%% HPM GYRO (2)
fprintf('\n-----------HPM GYRO---------------\n');

R_iADCS_C_HPM = dcm(2,-90/180*pi);
q_iADCS_C_HPM   = dcm2q(R_iADCS_C_HPM(:,:,1),'tsf','xyzw');

R_B_C_HPM     = R_B_iADCS*R_iADCS_C_HPM(:,:,1);
q_B_C_HPM       = dcm2q(R_B_C_HPM(:,:,1),'tsf','xyzw');

X_Y_Z_HPM        = [0.0;-1;-0.2];


% Report

fprintf('HPM Gyro [Instrument to iADCS] = [%.12f, %.12f, %.12f, %.12f]\n',q_iADCS_C_HPM);
fprintf('HPM Gyro [Instrument to Body] = [%.12f, %.12f, %.12f, %.12f]\n',q_B_C_HPM);


% Plot
plotvector(R_B_C_HPM*[1 ;0 ;0],X_Y_Z_HPM, [0.25 0.50 0.9], strcat('X_H_P_M',0.5));
plotvector(R_B_C_HPM*[0 ;1 ;0],X_Y_Z_HPM, [0.25 0.35 0.9], strcat('Y_H_P_M',0.25));
plotvector(R_B_C_HPM*[0 ;0 ;1],X_Y_Z_HPM, [0.25 0.20 0.9], strcat('Z_H_P_M',0.25)); 



