clear all; close all;
pkg load image


FileToRead='Rx2_data_adjusted.csv';
Heatmap=dlmread(FileToRead);
[X,Y] = meshgrid((0:360),(-90:90)); %original Matrix Koordianten bauen
[XI,YI] = meshgrid((0:0.1:360),(-90:0.1:90)); %größere Matrix Koordianten bauen
ZI = interp2(X,Y,Heatmap,XI,YI); %originale matrix auf größere interpolieren wenn Heatmap 181x361 anstelle von 361x181
##
hFig3=figure(3)
##set(hFig, 'Position', [0 0 1600 800])
set(hFig3, 'paperunits', 'normalized')
set(hFig3, 'paperposition', [0 0 1 1])
clf

imagesc((0:360),(-100:100),imsmooth(ZI,'gaussian'))
colormap;
axis tight

grid on 
title('Cancellation Pattern Rx2')
	
	set(gca, 'YAxisLocation', 'left')
	xlabel('Azimuth [deg]','FontSize',12)
	set(gca,'xtick',[0:30:360])

	set(gca,'Ydir','Normal')
	
	set(gca,'ytick',[-90:30:90])
	ylabel('Elevation [deg]','FontSize',12)


%%%%%
  %BST adapted colorbar for Octave (AW 20.11.18)
  File = 'parula.csv';
  values= dlmread(File);
  colormap(values);
  colorbar;
  h = colorbar('SouthOutside');
  xlabel(h,'Absolute Antenna Gain [dB]');
  %%%
 

print('\\bst-server3c\Projekte\Systeme\STM_Lagari\03-Documents\TR-Test-Reports\TR003_TTC_AntennaCancellation\src\2018-12-06_Results_IQWireless_Trafo\Cancellation_Pattern_Rx2.pdf','-S720,360') % erzeugt ein PDF mit 72 DPI mit 10x5 inch Groesse
clear FileToRead;