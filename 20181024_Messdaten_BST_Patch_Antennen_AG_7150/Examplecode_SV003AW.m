clear all; 
% close all; 
clc;
% graphics_toolkit('fltk')
load('BST_2_Patchantennen.mat');
% pkg load mapping;
dynamic = 30; % dB
freq =1; % Vektorindex



% uncal pattern combined / total
p1 = abs(pattern.value.uncal(:,:,freq,1));
p2 = abs(pattern.value.uncal(:,:,freq,2));
p = sqrt((p1 .* p1) + (p2 .* p2));

% normiertes pattern
plin = p ./ max(max(p(:,:))); % 0 bis 1
plog = 20*log10(  max(plin, 10^(-dynamic/20))  )    + dynamic; % 0dB bis dynamic

% directivity
PowerPatt = plin.* plin; % voltage pattern -> power pattern
Summe = 0; 
thetapts = length(pattern.dim(1).value);
phipts = length(pattern.dim(2).value);
for j = 1:phipts,
  
    for i = 1:thetapts,
        Summe = Summe + (PowerPatt(j,i) * abs(sin(  (pattern.dim(1).value(i))/180*pi  ))) ; %Betrag vom Sinus notwendig wenn theta span -180� bis + 180�
    end
end
Prad = 2*pi/phipts * pi/thetapts * Summe;
umax=max(max(abs(plin)));

Dlin = 4*pi*max(max(PowerPatt)) / Prad;
Dlog = 10*log10(Dlin);



% Trafo in Matlab-surf's kartesisches Koordinatensystem
[theta,phi] = meshgrid(pattern.measurement.dim(2).value, pattern.measurement.dim(1).value-90); % f�r sph2cart theta und phi vertauschen und 90�-offset: siehe Koord.-Definition in >>doc sph2cart
[x,y,zz] = sph2cart(theta*pi/180, phi*pi/180, plog);

%%% Transformation in BST Sat Frame $$$$ (27.11.18)
% for 1,1
for i = 1:1:121
    for j=1:1:61
        vector_test(:,i,j) = [x(i,j);y(i,j);zz(i,j)];
        vector_sc(:,i,j)   = dcm(1,pi/2)*dcm(3,-pi/2)*vector_test(:,i,j);
        
        x_sc(i,j) = vector_sc(1,i,j);
        y_sc(i,j) = vector_sc(2,i,j);
        z_sc(i,j) = vector_sc(3,i,j);

    end
end

x = x_sc;
y = y_sc;
zz = z_sc;


% cal pattern combined / total
pc1 = abs(pattern.value.cal(:,:,freq,1));
pc2 = abs(pattern.value.cal(:,:,freq,2));
pc = sqrt((pc1 .* pc1) + (pc2 .* pc2));


% Gmax �ber allen theta und phi der ausgew�hlten Frequenz
Glin = (max(max(pc(:,:))))^2;
Glog = max(max(20*log10(pc(:,:))));

% Effizienz
effeciency = Glin / Dlin;


% kalibrierte Daten f�r colorbar, mit dynamic
zcal = max(20*log10(pc(:,:)), Glog-dynamic); 


% P L O T
% macht gro�es figure-Fenster
scrsz = get(0,'ScreenSize');
figure('Position',[5 (scrsz(4)/25) scrsz(3)/1 scrsz(4)/1.13]);

% unsichtbarer plot mit kalibrierten Z-Werten f�r colorbar
h1 = surf(x, y, zcal); hold on;
axis equal; axis off;


  %%%%%
  %BST adapted colorbar for Octave (AW 20.11.18)
  File = 'parula.csv';
  values= dlmread(File);
  colormap(values);
  colorbar;
  %%%

%h2 = colorbar('peer', gca, 'location', 'eastoutside','fontname','arial','fontsize',18); 
colorbar;
set(get(colorbar,'title'),'String', 'gain [dBi]', 'FontSize', 18, 'fontname', 'arial'); 
caxis manual; 
set(h1,'visible','off');


% dargestellter dynamic-plot
axes;
h3 = surf(-x,-y,-zz, sqrt((x.*x)+(y.*y)+((zz).*(zz)))); hold on;

set(h3,'edgecolor','none'); 
shading interp;
axis equal; axis off;

% Koordinatensystem
xachse=plot3([0 1.2*round(max(max(x)))],[0 0],[0 0],'LineWidth',2, 'color', 'black');
xtext=text([1.2*round(max(max(x)))+0.8],[0],[0],'x','fontname','arial','FontSize',18);
yachse=plot3([0 0],[0 1.2*round(max(max(y)))+15.0],[0 0],'LineWidth',2, 'color', 'black');
ytext=text([0],[1.2*round(max(max(y)))+15.8],[0],'y','fontname','arial','FontSize',18);
zachse=plot3([0 0],[0 0],[0 dynamic+1],'LineWidth',2, 'color', 'black');
ztext=text([0],[0],[dynamic+1.8],'z','fontname','arial','FontSize',18);  

view(120,20);


usedfreq = pattern.dim(3).value(freq);
% title([aut.name,', ' num2str(usedfreq),' MHz',', Gmax = ',num2str(roundn(Glog,-1)),' dBi'], 'fontname', 'arial', 'fontsize', 20);
%title(['5-miniSMP, ' num2str(usedfreq),' GHz',', Gmax = ',num2str(roundn(Gmax,-1)),' dBi'], 'fontname', 'arial', 'fontsize', 22);
%text('Dmax = ',Dlog);

rotate3d on; 

figure (2)
h4 = surf(pattern.measurement.dim(1).value,pattern.measurement.dim(2).value,20*log10(pc(:,:)'), 'EdgeColor','none');
view([0 90])
%colormap(values);
colorbar;
ax1 = gca;
set(ax1, 'XLim', [pattern.measurement.dim(1).value(end) pattern.measurement.dim(1).value(1)]);
set(gca, 'xtick', [-180:90:180]);
%ax1.XLim = [pattern.measurement.dim(1).value(end) pattern.measurement.dim(1).value(1)];
set(ax1, 'YLim', [pattern.measurement.dim(2).value(1) pattern.measurement.dim(2).value(end)]);
set(gca, 'ytick', [0:45:180]);
%ax1.YLim = [pattern.measurement.dim(2).value(1) pattern.measurement.dim(2).value(end)];
set(ax1, 'CLim', [Glog-30 Glog]);
% %ax1.CLim = [Glog-30 Glog];
% set(ax1, 'XLabel', '\\theta (deg)');
% %ax1.XLabel.String  = '\theta (deg)';
% set(ax1, 'YLabel', '\\phi (deg)');
% %ax1.YLabel.String  = '\phi (deg)';
% set(ax1, 'Title', 'BST Patchantenna, 2230MHz, Gnax= 6dBi, Combined Gain (\\theta,\\phi) in dBi');
%ax1.Title.String   = ['Kombinierter Gewinn (\theta,\phi) in dBi'];
%print('C:\Users\wiedfeld\Desktop\2D_pattern.pdf','-S720,360');


%%%%%Mercator of the sphere%%%% (AW 27.11.18)
figure(4)
h5 = surf(theta, phi-90, zcal);
colorbar;
set(h5,'edgecolor','none'); 
shading interp;
view([0 90]);
ax1 = gca;
% set(ax1, 'XLim', [pattern.measurement.dim(1).value(end) pattern.measurement.dim(1).value(1)]);
% set(gca, 'xtick', [0:45:180]);
% set(ax1, 'YLim', [pattern.measurement.dim(2).value(1) pattern.measurement.dim(2).value(end)]);
% set(gca, 'ytick', [0:-90:-360]);
%set(ax1, 'CLim', [Glog-30 Glog]);
 set(ax1, 'XLabel', '\theta (deg)');
 set(ax1, 'YLabel', '\phi (deg)');
% set(ax1, 'Title', 'BST Patchantenna, 2230MHz, Gnax= 6dBi, Combined Gain (\\theta,\\phi) in dBi');