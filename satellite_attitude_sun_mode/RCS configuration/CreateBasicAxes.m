function CreateBasicAxes(radius, cell_labels)
% Create basic 3D figure window & add axes.
% Format:
%   [h_mainaxes] = CreateBasicAxes(radius, cell_labels, st_title)
% Inputs:
%   radius          radius of wireframe sphere generated
%   cell_labels     {3x1} array of axes label strings - e.g. {'X','Y','Z'}
% Outputs:
%   h_mainaxes      returns a handle to the created axes object

axis_colour = [1 1 1];
setup_figure('k');
plot_frame_axes(cell_labels,axis_colour,radius);
hold on
setup_fig_axes(1.5*radius);

% add transparent sphere with radius of "radius"
[x, y, z]    = sphere(20);
ps           = surf(radius*x,radius*y,radius*z);
set(ps,'facecolor','none','edgecolor',0.7*[1 1 1],'LineStyle',':');  

return

% =========================================================================
% supporting functions
% =========================================================================
function handle = setup_figure(colour)
screensize = get(0,'ScreenSize');           % Get the screensize 
handle     = figure;                        % Open a new figure and store the handle to it
% set the properties of the new figure
set(handle,'Color',colour,'UserData',1,'Position',[50 50 screensize(3)-100 screensize(4)-150]);
cameratoolbar('SetMode','orbit')
set(handle,'KeyPressFcn',@animation_keypress_callback)
return


function setup_fig_axes(scale)
set(gca,'Position',[0 0 1 1],'PlotBoxAspectRatioMode','manual','DataAspectRatioMode','manual','CameraViewAngleMode','manual');
set(gca,'Visible','off');
axis(scale*[-1 1 -1 1 -1 1]);
set(gca,'CameraViewAngle',6.5);
set(gca,'PlotBoxAspectRatio',[1 1 1],'DataAspectRatio',[1 1 1]);axis fill;
return


function handle = plot_frame_axes(axis_strings,colour,axis_size)
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
