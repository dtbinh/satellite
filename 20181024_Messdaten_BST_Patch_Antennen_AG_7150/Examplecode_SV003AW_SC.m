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
plog =-( 20*log10(  max(plin, 10^(-dynamic/20))  )    + dynamic); % 0dB bis dynamic

% directivity
PowerPatt = plin.* plin; % voltage pattern -> power pattern
Summe = 0; 
thetapts = length(pattern.dim(1).value);
phipts = length(pattern.dim(2).value);

for j = 1:phipts
  
    for i = 1:thetapts
        Summe = Summe + (PowerPatt(j,i) * abs(sin(  (pattern.dim(1).value(i))/180*pi  ))) ; %Betrag vom Sinus notwendig wenn theta span -180??? bis + 180???
    end
end
Prad = 2*pi/phipts * pi/thetapts * Summe;
umax=max(max(abs(plin)));

Dlin = 4*pi*max(max(PowerPatt)) / Prad;
Dlog = 10*log10(Dlin);

% Trafo in Matlab-surf's kartesisches Koordinatensystem
% theta - ele
% phi   - azi
[phi_int,theta_int] = meshgrid(pattern.measurement.dim(2).value, pattern.measurement.dim(1).value - 90 ); 
% [x,y,z] = sph2cart(azimuth,elevation,r)

[x_int,y_int,z_int] = sph2cart(phi_int*pi/180,theta_int*pi/180, plog); %[x,y,z] = sph2cart(azimuth,elevation,r)

fig = figure;
surf(phi_int,theta_int+90,-plog);
set(fig,'Name','RAW');
xlabel('phi [deg]');
ylabel('theta [deg]');

% ------------------------------------------------------------------------
% Obtain Measurement Data
[azi_int,elev_int,r] = cart2sph(x_int,y_int,z_int); % THis function is not 100%

% Refine Data
for i = 1:121
    for j = 1:61
        azi(i,j) = azi_int(2,j)*180/pi;
        elev(i,j) = elev_int(i,61)*180/pi;
    end
end


% Set Plot
di = 121;  % elev (121) -90 to 90  180 deg axis -> -z_axis -x direction
dj = 61;   %  azi (61) -180 to 180 360 deg axis -> -x_axis x-y plane +z direction

% Mapping
for i = 1:1:61   
    for j=1:1:61
        azi(i,j) = azi(i,j);
        elev(i,j) = pi/2 - elev(i,j);
    end
end


for i = 1:1:121      % ele
    for j=1:1:61   % azi
        fprintf('[%3d,%3d] azi: %8.2f ele: %8.2f\n',...
            i,j,azi(i,j),elev(i,j));
        
        % Rotate Vector
        vector_test(:,i,j)   = [x_int(i,j);y_int(i,j);z_int(i,j)];
        vector_sc_int(:,i,j) = dcm(1,pi/2)*dcm(3,-pi/2)*vector_test(:,i,j);       % BST
        vector_sc(:,i,j)     = dcm(3,-pi/2)*dcm(1,-pi/2)*vector_sc_int(:,i,j);    % STM
        
        % Test Frame
        x_test(i,j) = vector_test(1,i,j);
        y_test(i,j) = vector_test(2,i,j);
        z_test(i,j) = vector_test(3,i,j);
        
        mag_test(i,j)  = sqrt( x_test(i,j)^2 + y_test(i,j)^2 + z_test(i,j)^2);
        
        % SpaceCraft Frame
        x_sc(i,j) = vector_sc(1,i,j);
        y_sc(i,j) = vector_sc(2,i,j);
        z_sc(i,j) = vector_sc(3,i,j);
        
        mag_sc(i,j)  = sqrt( x_sc(i,j)^2 + y_sc(i,j)^2 + z_sc(i,j)^2);    
    end   
    fprintf('\n');
end

% -------------------------------------------------------------------------
% Plot
type ='test';

switch type
  case'test'
    xx  = x_test;
    yy  = y_test;
    zz  = z_test;   
    mag = mag_test;
  case 'body'
    xx  = x_sc;
    yy  = y_sc;
    zz  = z_sc;
    mag = mag_sc;
  otherwise
end


% cal pattern combined / total
pc1 = abs(pattern.value.cal(:,:,freq,1));
pc2 = abs(pattern.value.cal(:,:,freq,2));
pc  = sqrt((pc1 .* pc1) + (pc2 .* pc2));

% Gmax 
Glin = (max(max(pc(:,:))))^2;
Glog = max(max(20*log10(pc(:,:))));

% Efficiency
effeciency = Glin / Dlin;

% Magnitude Calibration
zcal = max(20*log10(pc(:,:)), Glog-dynamic); 

% Plot
screensize = get(0,'ScreenSize');

fig = figure;
set(fig,'Position',[screensize(3)*0.5 screensize(4)*0.25  screensize(3)*0.5 screensize(4)*0.5]);
set(fig,'Name',upper(type));

h3 = surf(xx,yy,zz); hold on;

axis equal; axis off;
cameratoolbar('SetMode','orbit')   
cameratoolbar('Show')
cameratoolbar('SetCoordSys','none')
set(gca,'CameraViewAngle',8); % Set Zoom of Graph
set(get(colorbar,'title'),'String', 'gain [dBi]', 'FontSize', 18, 'fontname', 'arial'); 
caxis manual; 
set(h3,'edgecolor','none'); 
% shading interp;



% Koordinatensystem
length = 50;
width  = 1;
xachse = plot3([0 length],[0 0],[0 0],'LineWidth',width, 'color', 'black');
xtext  = text([length],[0],[0],'x','fontname','arial','FontSize',18);

yachse = plot3([0 0],[0 length],[0 0],'LineWidth',width, 'color', 'black');
ytext  = text([0],[length],[0],'y','fontname','arial','FontSize',18);

zachse = plot3([0 0],[0 0],[0 length],'LineWidth',width, 'color', 'black');
ztext  = text([0],[0],[length],'z','fontname','arial','FontSize',18);  

% view(120,20);


usedfreq = pattern.dim(3).value(freq);
% title([aut.name,', ' num2str(usedfreq),' MHz',', Gmax = ',num2str(roundn(Glog,-1)),' dBi'], 'fontname', 'arial', 'fontsize', 20);
%title(['5-miniSMP, ' num2str(usedfreq),' GHz',', Gmax = ',num2str(roundn(Gmax,-1)),' dBi'], 'fontname', 'arial', 'fontsize', 22);
%text('Dmax = ',Dlog);

% rotate3d on; 

% [X,Y] = meshgrid(0.5:0.5:5,1:20);
% % X - 20x10
% % Y - 20x10
% % Z - 20x10
% Z = sin(X) + cos(Y);
% figure
% surf(X,Y,Z)
return
%% ---------------------------------------------------------------------------
% 2D Plot
fig = figure;
% x_data = pattern.measurement.dim(1).value;
% y_data = pattern.measurement.dim(2).value;
% z_data = 20*log10(pc(:,:)');

% Set plot
x_data = azi*180/pi;  % 
y_data = elev*180/pi;    % 
z_data = mag;

% set(fig,'Position',[screensize(3)*0.5 screensize(4)*0.5  screensize(3)*0.5 screensize(4)*0.5]);
h4 = surf(x_data,y_data, z_data, 'EdgeColor','none');
xlabel('Elevation: theta 0 to 180');
ylabel('Azimuth:   phi -180 to 180');
zlabel('Magnitude');
view([0 -90])


return


% set(gca, 'XLim', [x_data y_data]);
set(gca, 'xtick', y_data);

% set(get(colorbar,'title'),'String', 'gain [dBi]', 'FontSize', 18, 'fontname', 'arial'); 

%ax1.XLim = [pattern.measurement.dim(1).value(end) pattern.measurement.dim(1).value(1)];
% set(gca, 'YLim', [x_data y_data]);
x_data = x_data';
set(gca, 'ytick', x_data);
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
