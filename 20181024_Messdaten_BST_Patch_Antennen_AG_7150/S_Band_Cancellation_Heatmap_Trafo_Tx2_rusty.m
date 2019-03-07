clear all; 
close all;
clc

% Raw Data
FileToRead='Tx2_data_adjusted.csv';
Heatmap = dlmread(FileToRead);
[XI,YI] = meshgrid((0:1:360),(-90:1:90)); %gr????ere Matrix Koordianten bauen

% for row=1:length(Heatmap(:,1))
%     for col=1:180
%         ZI(row,col)      = Heatmap(row,col+180);
%     end
%     
%     for col=181:length(Heatmap(1,:))
%         ZI(row,col) = Heatmap(row,col-180);
%     end
% end
for row=1:length(Heatmap(:,1))
     for col=1:length(Heatmap(1,:))
         ZI(row,col)= Heatmap(row,362-col);

     end
end

[azi,ele] = meshgrid((0:1:360),(0:1:180)); %gr????ere Matrix Koordianten bauen

% ------------------------------------------
azi_raw =  XI;
ele_raw =  YI;
mag_raw =  ZI;

figure
splot = surf(azi,ele,mag_raw);
set(splot,'edgecolor','none');
view([0 90]);
ylabel('Elevation from Z axis');
xlabel('Azimuth from X-Y plane from X-axis in Z-direction');
set(get(colorbar,'title'),'String', 'gain [dBi]', 'FontSize', 18, 'fontname', 'arial'); 

[x_int,y_int,z_int] = bst_sph2cart(azi*pi/180, ele*pi/180, -mag_raw*1000); 

k = 1;
% Plot sections
for i = 1:1:180
    for j = 1:1:180
        
        V(1,k) = x_int(i,j);
        V(2,k) = y_int(i,j);
        V(3,k) = z_int(i,j);
        
        x(i,j) = x_int(i,j);
        y(i,j) = y_int(i,j);
        z(i,j) = z_int(i,j);
        
        k=k+1;
    end
end

figure
surf(x_int,y_int,z_int);
axis equal;
xlabel('x');
ylabel('y');
zlabel('z');
xlim([-10.0 10.0]);
ylim([-10.0 10.0]);
zlim([-10.0 10.0]);



return
hFig3 = figure(3)
set(hFig3, 'Position', [0 0 1600 800])
set(hFig3, 'paperunits', 'normalized')
set(hFig3, 'paperposition', [0 0 1 1])
clf

% imagesc((0:360),(-100:100),imsmooth(ZI,'gaussian'))
imagesc((0:360),(-90:90),ZI)
colormap;
axis tight

grid on 
title('Cancellation Pattern Tx2')
	
	set(gca, 'YAxisLocation', 'left')
	xlabel('Azimuth [deg]','FontSize',12)
	set(gca,'xtick',[0:30:360])

	set(gca,'Ydir','Normal')
	
	set(gca,'ytick',[-90:30:90])
	ylabel('Elevation [deg]','FontSize',12)