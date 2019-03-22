function plot3d(x, y, z, mag, cscale,cname)



if ~exist('cname','var')
    cname = 'gain [dBi]';
end

% Get Maximum
maximum = max(max(sqrt(x.^2+y.^2+z.^2)));

% Plot

splot = surf(x,y,z,mag); 
axis equal; axis off;hold on;

set(gca,'CameraViewAngle',8); % Set Zoom of Graph
set(get(colorbar,'title'),'String', cname, 'FontSize', 10, 'fontname', 'arial'); 
set(splot,'edgecolor','none');
cameratoolbar('SetMode','orbit')   
cameratoolbar('Show')
cameratoolbar('SetCoordSys','none')


if exist('cscale','var')
   caxis(cscale);
end


% shading interp;
% caxis manual; 


% Koordinatensystem
length = 1.5*maximum;
width  = 1;
xachse = plot3([0 length],[0 0],[0 0],'LineWidth',width, 'color', 'black');
xtext  = text(length*1.1,0,0,'x','fontname','arial','FontSize',18);

yachse = plot3([0 0],[0 length],[0 0],'LineWidth',width, 'color', 'black');
ytext  = text(0,length*1.1,0,'y','fontname','arial','FontSize',18);

zachse = plot3([0 0],[0 0],[0 length],'LineWidth',width, 'color', 'black');
ztext  = text(0,0,length*1.1,'z','fontname','arial','FontSize',18);  
end