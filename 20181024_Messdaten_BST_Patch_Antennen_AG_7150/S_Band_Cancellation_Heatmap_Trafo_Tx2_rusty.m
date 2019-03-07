clear all; 
close all;
clc

% Raw Data
FileToRead='Tx2_data_adjusted.csv';
Heatmap = dlmread(FileToRead);

for row=1:length(Heatmap(:,1))
     for col=1:length(Heatmap(1,:))
         ZI(row,col)= Heatmap(row,362-col);
     end
end

[XI,YI] = meshgrid((0:1:360),(0:1:180)); 
% ------------------------------------------
% IQ Wireless Coordinates (Converted in Spherical Coordinates)

azi_raw =  XI;
ele_raw =  YI;
mag_raw =  10.^(ZI/20);

figure
splot = surf(azi_raw,ele_raw,ZI);
set(splot,'edgecolor','none');
view([0 90]);
ylabel('Elevation from Z axis');
xlabel('Azimuth from X-Y plane from X-axis in Z-direction');
set(get(colorbar,'title'),'String', 'gain [dBi]', 'FontSize', 18, 'fontname', 'arial'); 
cscale = [-13 3.0];
caxis(cscale);


% Plot 3D
[x_int,y_int,z_int] = bst_sph2cart(azi_raw*pi/180, ele_raw*pi/180, mag_raw); 
for i = 1:1:length(x_int(:,1))
    for j=1:1:length(x_int(1,:))
        vector_test(:,i,j) = [x_int(i,j);y_int(i,j);z_int(i,j)];
    end
end

plot3d(x_int,y_int,z_int,ZI, 'test',cscale ); 

% ------------------------------------------------------------------------
% Spacecraft Coordinates
R_test_sc = dcm(3,pi/2)*dcm(1,pi/2);

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

r_sc = 20*log10(mag_sc);

% Plot 2D
[xq,yq,vq] = plotinterpolate(azi_sc,elev_sc,r_sc);
plot2d(xq*180/pi, yq*180/pi, vq, [0 90], 'SPACECRAFT',cscale);

% Plot 3D
plot3d(x_sc,y_sc,z_sc,r_sc, 'spacecraft',cscale); 

