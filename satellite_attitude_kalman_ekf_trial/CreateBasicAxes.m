function CreateBasicAxes(radius, cell_labels)
close all
% Create basic 3D figure window & add axes.
% Format:
%   [h_mainaxes] = CreateBasicAxes(radius, cell_labels, st_title)

% Inputs:
%   radius          radius of wireframe sphere generated
%   cell_labels     {3x1} array of axes label strings - e.g. {'X','Y','Z'}
% Outputs:
%   h_mainaxes      returns a handle to the created axes object

if ~exist('radius','var')
    radius = 1;
end
if ~exist('cell_labels','var')
    cell_labels = {'X' 'Y' 'Z'};
end


axis_colour = [1 1 1]; % Axis Color
setup_figure('k');     % Background Color
plot_frame_axes(cell_labels,axis_colour,radius); % set up frame axis
hold on
setup_fig_axes(1.5*radius);

% CREATE SPHERE
[x, y, z] = sphere(20); 
ps        = surf(radius*x,radius*y,radius*z);
set(ps,'facecolor','none','edgecolor',0.7*[1 1 1],'LineStyle',':');  

return

% =========================================================================
% supporting functions
% =========================================================================
function handle = setup_figure(colour)
screensize = get(0,'ScreenSize');           % Get the screensize [ 1  1        1280         720]
handle     = figure;                        % Open a new figure and store the handle to it

set(handle,'Color',colour,'UserData',1)                   % set the color and user of the new figure
set(handle,'Position',[0 0 screensize(4) screensize(4)]); % set position and size of figure
set(handle,'KeyPressFcn',@animation_keypress_callback)    % 
cameratoolbar('SetMode','orbit')                          % set rotatable figure
return


function setup_fig_axes(scale)
set(gca,'Position',[0 0 1 1]);
set(gca,'PlotBoxAspectRatioMode','manual','DataAspectRatioMode','manual','CameraViewAngleMode','manual');
set(gca,'Visible','off');
axis(scale*[-1 1 -1 1 -1 1]); % Axis Scale Setting
set(gca,'CameraViewAngle',5); % Zoom
set(gca,'PlotBoxAspectRatio',[1 1 1],'DataAspectRatio',[1 1 1]);
axis fill;
return


function handle = plot_frame_axes(axis_strings,colour,axis_size);
for i=1:3
    v    = [i==1 i==2 i==3]';
    h(1) = line([-v(1) +v(1)],[-v(2) +v(2)],[-v(3) +v(3)],'Color',colour);
    h(2) = text(1.1*v(1), 1.1*v(2), 1.1*v(3), axis_strings{i}, 'Color',colour);
    eval(['handle.h_v' num2str(i) ' = h;']);
end
return


function animation_keypress_callback(h_fig,eventobj)
if(strcmp(get(h_fig,'CurrentCharacter'),' '))
    fig_flag = get(h_fig,'UserData');
    fig_flag = ~fig_flag;
    set(h_fig,'UserData',fig_flag);
end
