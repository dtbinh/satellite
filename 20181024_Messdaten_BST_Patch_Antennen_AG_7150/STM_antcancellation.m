close all
clear all
clc

% ------------------------------------------------------------------------
% LOAD DATA
AntType = 'TX2';
file   = strcat(AntType,'.txt');
fileID = fopen(file);
head  = textscan(fileID,'%150c',1);
head  = textscan(fileID,'%150c',1);
data  = textscan(fileID,'%f %f %f %f %f %f %f %f');
fclose(fileID);
% ------------------------------------------------------------------------
% INPUT PARAMETER
theta  = data{1};   % Theta [deg.]  
phi    = data{2};   % Phi   [deg.]
dir    = data{3};   % Abs(Dir.)[dBi   ]
abs_left    = data{4};   % Abs(Left )[dBi   ] 
phase_left  = data{5};   % Phase(Left )[deg.]
abs_right   = data{6};   % Abs(Right)[dBi   ]
phase_right = data{7};   % Phase(Right)[deg.]
ax_ratio    = data{8};   % Ax.Ratio[dB    ]

% ------------------------------------------------------------------------
% DATA PROCESSING

azi = phi;
ele = theta;
mag = dir;

min_azi = min(phi);
max_azi = max(phi);
stp_azi = 1;

min_ele = min(theta);
max_ele = max(theta);
stp_ele = 1;

grid_azi = min_azi:stp_azi:max_azi;
grid_ele = min_ele:stp_ele:max_ele;

% Polar
[azi_q,ele_q] = meshgrid(grid_azi,grid_ele );
mag_q         = griddata(azi,ele,mag,azi_q,ele_q);     % dBi

mag_watt =  10.^(mag_q/20); % [watt}

% Cartesian
[x_sc,y_sc,z_sc] = bst_sph2cart(azi_q*pi/180,ele_q*pi/180,mag_watt);

% ------------------------------------------
% Plot
screensize = get(0,'ScreenSize');
fig = figure;
set(fig,'Name','STM - SPACECRAFT');
set(fig,'Position',[screensize(3)*0.5 screensize(4)*0.25  screensize(3)*0.75 screensize(4)*0.5]);
sub2D = subplot(1,2,1);
plot2d(azi_q, ele_q, mag_q, [0 90]);
sub3D = subplot(1,2,2);
plot3d(x_sc,y_sc, z_sc, mag_q); 

% --- Print---
pos = get(fig,'Position');
set(fig,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3)/100, pos(4)/100])
print(fig,strcat(AntType,'.pdf'),'-dpdf')
print(fig,strcat(AntType,'.jpeg') ,'-djpeg')

subfig2D = figure;
fig_2D = copyobj(sub2D, subfig2D);
set(fig_2D, 'Position', get(0, 'DefaultAxesPosition'));
set(get(colorbar,'title'),'String', 'gain [dBi]', 'FontSize', 10, 'fontname', 'arial');
print(subfig2D,strcat(AntType,'_2D','.jpeg') ,'-djpeg')

subfig3D = figure;
fig_3D = copyobj(sub3D, subfig3D);
set(fig_3D, 'Position', get(0, 'DefaultAxesPosition'));
set(get(colorbar,'title'),'String', 'gain [dBi]', 'FontSize', 10, 'fontname', 'arial');
print(subfig3D,strcat(AntType,'_3D','.jpeg') ,'-djpeg')




