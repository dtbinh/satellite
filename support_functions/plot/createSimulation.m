function createSimulation(pos,zoom)
% Figure Setting
fig = figure;
screensize = get(0,'ScreenSize');
set(fig,'Position',[screensize(3)*0 0 screensize(4)*0.8 screensize(4)*1]);
grid on; axis fill;
cameratoolbar('SetMode','orbit')   
set(gca,'Position',pos); % Set Position of Graph
set(gca,'CameraViewAngle',zoom);  % Set Zoom of Graph
axis(3.5*[-1 1 -1 1 -1 1]);    % Set Limit of Axis 

% Inertial Frame
X_eci = plotvector([1 ;0 ;0], [0 0 0], 'k', 'X_e_c_i');
Y_eci = plotvector([0 ;1 ;0], [0 0 0], 'k', 'Y_e_c_i');
Z_eci = plotvector([0 ;0 ;1], [0 0 0], 'k', 'Z_e_c_i');
hold on;

% Create Sphere
radius  = 1;
[x,y,z] = sphere(20); 
earth   = surf(radius*x,radius*y,radius*z);
set(earth,'facecolor','none','edgecolor',0.7*[1 1 1],'LineStyle',':'); 
end