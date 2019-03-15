clear all; 
%close all; 
clc;

% Load Data
load('BST_2_Patchantennen.mat');

% uncal pattern combined / total
freq    = 1;  % Vektorindex
p1 = abs(pattern.value.uncal(:,:,freq,1));
p2 = abs(pattern.value.uncal(:,:,freq,2));
p  = sqrt((p1 .* p1) + (p2 .* p2));


% normiertes pattern
dynamic = 30; % dB
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


freq    = 1;  % Vektorindex
dynamic = 30;
pc1 = abs(pattern.value.cal(:,:,freq,1));
pc2 = abs(pattern.value.cal(:,:,freq,2));
pc = sqrt((pc1 .* pc1) + (pc2 .* pc2));


% Gmax ???ber allen theta und phi der ausgew???hlten Frequenz
Glin = (max(max(pc(:,:))))^2;
Glog = max(max(20*log10(pc(:,:))));

zcal = max(20*log10(pc(:,:)), Glog-dynamic); 


% ------------------------------------------------------------------------
% Obtain the phi and theta mesh grid
[phi_int,theta_int] = meshgrid(pattern.measurement.dim(2).value, pattern.measurement.dim(1).value - 90 ); 
% [x,y,z] = sph2cart(azimuth,elevation,r)



for i=1:121
    for j =1:61
        if abs(plog(i,j)) < 1e-6
            plog(i,j) = -1e-6;
        end
    end
end

% Plot Surface
% plot2d(phi_int, theta_int, -plog, [0 90], 'RAW');

% ------------------------------------------------------------------------
% Obtain the Vectors
[x_int,y_int,z_int] = sph2cart(phi_int*pi/180,theta_int*pi/180, plog); %[x,y,z] = sph2cart(azimuth,elevation,r)

% ------------------------------------------------------------------------
% Obtain Measurement Data
[azi_int,elev_int,r_int] = bst_cart2sph(x_int,y_int,z_int); % THis function is not 100%

% plot2d(azi_int*180/pi, elev_int*180/pi, -plog, [0 -90], 'TEST');
% plot3d(x_int,y_int,z_int,r_int, 'test'); 


for i = 1:1:121      % ele
    for j=1:1:61   % azi
        
        % Rotate Vector
        vector_test(:,i,j) = [x_int(i,j);y_int(i,j);z_int(i,j)];
        vector_sc(:,i,j)   = dcm(1,pi/2)*dcm(3,-pi/2)*vector_test(:,i,j);       % BST
        vector_stm(:,i,j)     = dcm(3,-pi/2)*dcm(1,-pi/2)*vector_sc(:,i,j);    % STM
        
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
        
         % STM Frame
        x_stm(i,j) = vector_stm(1,i,j);
        y_stm(i,j) = vector_stm(2,i,j);
        z_stm(i,j) = vector_stm(3,i,j);
        
        mag_stm(i,j)  = sqrt( x_stm(i,j)^2 + y_stm(i,j)^2 + z_stm(i,j)^2);  
    end   
    fprintf('\n');
end

% -------------------------------------------------------------------------
% Test
% figure
% contour(azi_int*180/pi, elev_int*180/pi, r_int,70, 'Fill', 'on');
% 
% figure
% plotscatter(azi_int*180/pi,elev_int*180/pi,r_int);
% 
% plot2d(azi_int*180/pi, elev_int*180/pi, r_int, [0 -90], 'TEST');
% plot3d(x_test,y_test,z_test,mag_test, 'test'); 

% -----------------------------------------------------------------------
% Spacecraft
screensize = get(0,'ScreenSize');

[azi_sc,elev_sc,r_sc] = bst_cart2sph(x_sc,y_sc,z_sc);

% figure
% plotscatter(azi_sc*180/pi,elev_sc*180/pi,r_sc);
% 
% figure
% contour(azi_sc*180/pi, elev_sc*180/pi, r_sc,70, 'Fill', 'on');
% 
% plot2d(azi_sc*180/pi, elev_sc*180/pi, r_sc, [0 -90], 'SPACECRAFT');
fig = figure;
set(fig,'Name','MEASUREMENT - SPACECRAFT');
set(fig,'Position',[screensize(3)*0.5 screensize(4)*0.25  screensize(3)*0.75 screensize(4)*0.5]);
sub2D = subplot(1,2,1);
[xq,yq,vq] = plotinterpolate(azi_sc,elev_sc,zcal);
plot2d(xq*180/pi, yq*180/pi, vq,[0 90]);
sub3D = subplot(1,2,2);
plot3d(x_sc,y_sc,z_sc,zcal); 

% r_sc -- > zcal how?

Name = 'measurement_spacecraft';
% ----------------------------------------------------------
pos = get(fig,'Position');
set(fig,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3)/100, pos(4)/100])
print(fig,strcat(Name,'.pdf'),'-dpdf')
print(fig,strcat(Name,'.jpeg') ,'-djpeg')

subfig2D = figure;
fig_2D = copyobj(sub2D, subfig2D);
set(fig_2D, 'Position', get(0, 'DefaultAxesPosition'));
set(get(colorbar,'title'),'String', 'gain [dBi]', 'FontSize', 10, 'fontname', 'arial');
print(subfig2D,strcat(Name,'_2D','.jpeg') ,'-djpeg')

subfig3D = figure;
fig_3D = copyobj(sub3D, subfig3D);
set(fig_3D, 'Position', get(0, 'DefaultAxesPosition'));
set(get(colorbar,'title'),'String', 'gain [dBi]', 'FontSize', 10, 'fontname', 'arial');
print(subfig3D,strcat(Name,'_3D','.jpeg') ,'-djpeg')

