function plot3d(x, y, z, mag, name)

% Plot
screensize = get(0,'ScreenSize');
fig = figure;
splot = surf(x,y,z,mag); 
axis equal; axis off;hold on;
set(fig,'Position',[screensize(3)*0.5 screensize(4)*0.25  screensize(3)*0.5 screensize(4)*0.5]);
set(fig,'Name',upper(name));
set(gca,'CameraViewAngle',8); % Set Zoom of Graph
set(get(colorbar,'title'),'String', 'gain [dBi]', 'FontSize', 18, 'fontname', 'arial'); 
set(splot,'edgecolor','none');
cameratoolbar('SetMode','orbit')   
cameratoolbar('Show')
cameratoolbar('SetCoordSys','none')
caxis([-20 30])
% shading interp;
% caxis manual; 


% Koordinatensystem
length = 50;
width  = 1;
xachse = plot3([0 length],[0 0],[0 0],'LineWidth',width, 'color', 'black');
xtext  = text([length],[0],[0],'x','fontname','arial','FontSize',18);

yachse = plot3([0 0],[0 length],[0 0],'LineWidth',width, 'color', 'black');
ytext  = text([0],[length],[0],'y','fontname','arial','FontSize',18);

zachse = plot3([0 0],[0 0],[0 length],'LineWidth',width, 'color', 'black');
ztext  = text([0],[0],[length],'z','fontname','arial','FontSize',18);  
end