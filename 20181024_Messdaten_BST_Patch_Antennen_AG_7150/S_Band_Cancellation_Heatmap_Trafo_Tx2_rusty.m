clear all; 
close all;
clc

% Raw Data
AntType   = 'Tx2_data_adjusted';
FileToRead=strcat(AntType,'.csv');
Heatmap   = dlmread(FileToRead);


[XI,YI] = meshgrid((0:1:360),(0:1:180));
ZI      = Heatmap;
% ------------------------------------------
% IQ Wireless Coordinates (Converted in Spherical Coordinates)

azi_raw =  XI;
ele_raw =  YI;
mag_raw =  10.^(ZI/20); % [watt}

return
[x_int,y_int,z_int] = bst_sph2cart(azi_raw*pi/180, ele_raw*pi/180, mag_raw);

for i = 1:1:length(x_int(:,1))
    for j=1:1:length(x_int(1,:))
        vector_test(:,i,j) = [x_int(i,j);y_int(i,j);z_int(i,j)];
    end
end



% ------------------------------------------------------------------------
% Spacecraft Coordinates
switch FileToRead
    case 'Tx1_data_adjusted.csv'
        R_test_sc = dcm(1,-pi/2);
    case 'Tx2_data_adjusted.csv'
        R_test_sc = dcm(3,-pi/2)*dcm(1,-pi/2);
    
    otherwise
        R_test_sc = dcm(1,0);
end

for i = 1:1:length(x_int(:,1))
    for j=1:1:length(x_int(1,:))
        vector_sc(:,i,j)   = R_test_sc*vector_test(:,i,j); 
        
        % SpaceCraft Frame
        x_sc(i,j) = vector_sc(1,i,j);
        y_sc(i,j) = vector_sc(2,i,j);
        z_sc(i,j) = vector_sc(3,i,j);
        
        mag_sc(i,j) = sqrt( x_sc(i,j)^2 + y_sc(i,j)^2 + z_sc(i,j)^2);
    end
end

[azi_sc,elev_sc,r_sc] = bst_cart2sph(x_sc,y_sc,z_sc);

r_sc = 20*log10(mag_sc); %gain [dBi]

% ------------------------------------------------------------------------
% Plot
screensize = get(0,'ScreenSize');





fig = figure;
set(fig,'Name','IQ WIRELESS');
set(fig,'Position',[screensize(3)*0.5 screensize(4)*0.25  screensize(3)*0.75 screensize(4)*0.5]);
subplot(1,2,1)
plot2d(azi_raw, ele_raw, ZI, [0 90])

subplot(1,2,2)
plot3d(x_int,y_int,z_int,ZI); 

% Plot 2D
fig = figure;
set(fig,'Name','IQ WIRELESS - SPACECRAFT');
set(fig,'Position',[screensize(3)*0.5 screensize(4)*0.25  screensize(3)*0.75 screensize(4)*0.5]);
[xq,yq,vq] = plotinterpolate(azi_sc,elev_sc,r_sc);
sub2D=subplot(1,2,1);
plot2d(xq*180/pi, yq*180/pi, vq, [0 90]);
sub3D = subplot(1,2,2);
plot3d(x_sc,y_sc, z_sc, r_sc); 

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

