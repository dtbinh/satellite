clear all; 
close all; 
clc;

% Load Data
load('BST_2_Patchantennen.mat');

dynamic = 30; % dB
freq    = 1;  % Vektorindex

% uncal pattern combined / total
p1 = abs(pattern.value.uncal(:,:,freq,1));
p2 = abs(pattern.value.uncal(:,:,freq,2));
p  = sqrt((p1 .* p1) + (p2 .* p2));

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
        Summe = Summe + (PowerPatt(j,i) * abs(sin(  (pattern.dim(1).value(i))/180*pi  ))) ; %Betrag vom Sinus notwendig wenn theta span -180??? bis + 180???
    end
end
Prad = 2*pi/phipts * pi/thetapts * Summe;
umax=max(max(abs(plin)));

Dlin = 4*pi*max(max(PowerPatt)) / Prad;
Dlog = 10*log10(Dlin);

% Trafo in Matlab-surf's kartesisches Koordinatensystem
[theta,phi] = meshgrid(pattern.measurement.dim(2).value, pattern.measurement.dim(1).value-90); % f???r sph2cart theta und phi vertauschen und 90???-offset: siehe Koord.-Definition in >>doc sph2cart
[x,y,zz] = sph2cart(theta*pi/180, phi*pi/180, plog);

theta
phi
return
%

% Set
di = 121;
dj = 61;

for i = 1:1:di      % ' 360 phi - axial rotation angle of the positioner (azimuth of antenna)'
    for j=1:1:dj    % ' 180 theta - horizontal rotation angle of the positioner (elevation of antenna)'
        vector_test(:,i,j)   = [x(i,j);y(i,j);zz(i,j)];
        vector_sc_int(:,i,j) = dcm(1,pi/2)*dcm(3,-pi/2)*vector_test(:,i,j);   % BST
        vector_sc(:,i,j) = dcm(3,-pi/2)*dcm(1,-pi/2)*vector_sc_int(:,i,j);    % STM

        x_sc(i,j) = vector_sc(1,i,j);
        y_sc(i,j) = vector_sc(2,i,j);
        z_sc(i,j) = vector_sc(3,i,j);
        
        mag_int(i,j)  = sqrt( x_sc(i,j)^2 + y_sc(i,j)^2 + z_sc(i,j)^2);    
    end   
end

for i = 1:1:di      % 'phi - axial rotation angle of the positioner (azimuth of antenna)'
    for j=1:1:dj 
        y_data(j) =  180-(j-1)*(180/dj); 
    end
    x_data(i) =  -180+(i-1)*(360/di); 
end

% Mapping one
for i = 1:1:61     
    for j=1:1:61 
        mag2(i,j)    = mag_int(62-i,j);
        mag2(60+i,j) = mag_int(60+i,j);
        
        mag3(i,j)    = mag2(i,j);
        mag3(i,j+60) = mag2(60+i,j);
    end 
end

% Final Mapping
for i = 1:1:61    
    for j = 1:1:121
        mag(i,j) = mag3(62-i,122-j);
    end
end


x  = x_sc;
y  = y_sc;
zz = z_sc;


% cal pattern combined / total
pc1 = abs(pattern.value.cal(:,:,freq,1));
pc2 = abs(pattern.value.cal(:,:,freq,2));
pc = sqrt((pc1 .* pc1) + (pc2 .* pc2));

% Gmax ???ber allen theta und phi der ausgew???hlten Frequenz
Glin = (max(max(pc(:,:))))^2;
Glog = max(max(20*log10(pc(:,:))));

% Effizienz
effeciency = Glin / Dlin;

% kalibrierte Daten f???r colorbar, mit dynamic
zcal = max(20*log10(pc(:,:)), Glog-dynamic); 



% P L O T
% macht gro???es figure-Fenster
screensize = get(0,'ScreenSize');
fig = figure;
set(fig,'Position',[screensize(3)*0.5 0  screensize(3)*0.5 screensize(4)*0.5]);
cameratoolbar('SetMode','orbit')   
cameratoolbar('Show')
cameratoolbar('SetCoordSys','none')
set(gca,'CameraViewAngle',4); % Set Zoom of Graph
% unsichtbarer plot mit kalibrierten Z-Werten f???r colorbar
h1 = surf(x, y, zcal); hold on;
axis equal; axis off;

  %%%%%
  %BST adapted colorbar for Octave (AW 20.11.18)
%   File = 'parula.csv';
%   values= dlmread(File);
%   colormap(values);
%   colorbar;
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
xachse = plot3([0 50],[0 0],[0 0],'LineWidth',2, 'color', 'black');
xtext  = text([50],[0],[0],'x','fontname','arial','FontSize',18);

yachse = plot3([0 0],[0 50],[0 0],'LineWidth',2, 'color', 'black');
ytext  = text([0],[50],[0],'y','fontname','arial','FontSize',18);

zachse = plot3([0 0],[0 0],[0 50],'LineWidth',2, 'color', 'black');
ztext  = text([0],[0],[50],'z','fontname','arial','FontSize',18);  

view(120,20);


usedfreq = pattern.dim(3).value(freq);
% title([aut.name,', ' num2str(usedfreq),' MHz',', Gmax = ',num2str(roundn(Glog,-1)),' dBi'], 'fontname', 'arial', 'fontsize', 20);
%title(['5-miniSMP, ' num2str(usedfreq),' GHz',', Gmax = ',num2str(roundn(Gmax,-1)),' dBi'], 'fontname', 'arial', 'fontsize', 22);
%text('Dmax = ',Dlog);

rotate3d on; 

%% ---------------------------------------------------------------------------
% 2D Plot
fig = figure;
% x_data = pattern.measurement.dim(1).value;
% y_data = pattern.measurement.dim(2).value;
% z_data = 20*log10(pc(:,:)');

z_data = mag(:,:);

set(fig,'Position',[screensize(3)*0.5 screensize(4)*0.5  screensize(3)*0.5 screensize(4)*0.5]);
h4 = surf(x_data,y_data,  z_data, 'EdgeColor','none');
xlabel('x');
ylabel('y');
zlabel('Magnitude');
view([0 -90])
return




% set(gca, 'XLim', [x_data y_data]);
set(gca, 'xtick', x_data);


colormap(values);
colorbar;
% set(get(colorbar,'title'),'String', 'gain [dBi]', 'FontSize', 18, 'fontname', 'arial'); 

%ax1.XLim = [pattern.measurement.dim(1).value(end) pattern.measurement.dim(1).value(1)];
% set(gca, 'YLim', [x_data y_data]);
set(gca, 'ytick', y_data);
%ax1.YLim = [pattern.measurement.dim(2).value(1) pattern.measurement.dim(2).value(end)];


set(ax1, 'CLim', [Glog-30 Glog]);
% %ax1.CLim = [Glog-30 Glog];
 set(ax1, 'XLabel', '\theta (deg)');
% %ax1.XLabel.String  = '\theta (deg)';
 set(ax1, 'YLabel', '\phi (deg)');
% %ax1.YLabel.String  = '\phi (deg)';
 set(ax1, 'Title', 'BST Patchantenna, 2230MHz, Gnax= 6dBi, Combined Gain (\theta,\phi) in dBi');
%ax1.Title.String   = ['Kombinierter Gewinn (\theta,\phi) in dBi'];
%print('C:\Users\wiedfeld\Desktop\2D_pattern.pdf','-S720,360');

return

% filename= "test1.csv";
% fid = fopen(filename, "w");
% dlmwrite(fid,20*log10(pc));
% 
% ##%%%%%Angle Image%%%% (AW 27.11.18)
% ##figure(4)
% ##h5 = surf(phi,theta, z_sc);
% ##colorbar;
% ##set(h5,'edgecolor','none'); 
% ##shading interp;
% ##view([0 90])
% ##colormap(values);
% ##colorbar;
% ##ax1 = gca;
% ##%set(ax1, 'XLim', [pattern.measurement.dim(1).value(end) pattern.measurement.dim(1).value(1)]);
% ##set(gca, 'xtick', [-180:90:180]);S
% ##%ax1.XLim = [pattern.measurement.dim(1).value(end) pattern.measurement.dim(1).value(1)];
% ##%set(ax1, 'YLim', [pattern.measurement.dim(2).value(1) pattern.measurement.dim(2).value(end)]);
% ##set(gca, 'ytick', [0:45:180]);
% ##%ax1.YLim = [pattern.measurement.dim(2).value(1) pattern.measurement.dim(2).value(end)];
% ##set(ax1, 'CLim', [Glog-30 Glog]);
% ##% %ax1.CLim = [Glog-30 Glog];
% ## set(ax1, 'XLabel', '\theta (deg)');
% ##% %ax1.XLabel.String  = '\theta (deg)';
% ## set(ax1, 'YLabel', '\phi (deg)');
% ##% %ax1.YLabel.String  = '\phi (deg)';
% ## set(ax1, 'Title', 'BST Patchantenna, 2230MHz, Gnax= 6dBi, Combined Gain (\\theta,\\phi) in dBi');
% ##%ax1.Title.String   = ['Kombinierter Gewinn (\theta,\phi) in dBi'];
% ##%print('C:\Users\wiedfeld\Desktop\2D_pattern.pdf','-S720,360');