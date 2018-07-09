function h_axes = Vis3DAddFrameAxes(frame_origin, axis_labels, axis_colour, axis_size, h_axes)
% Vis3DAddFrameAxes. Add frame axes vectors & labels to current figure window
% Format:
%   Vis3DAddFrameAxes(frame_origin, axis_labels, axis_colour, axis_size)
% Inputs:
%   frame_origin    position of frame origin in current figure window
%   axis_labels     {3x1} array of axes label strings
%   axis_colour     colour of axes
%   axis_size       length of axes
%   h_axes          handle of current axes (in MATLAB sense)
% Outputs:
%   h_axes          handles to 3D plot objects


% change to specified axes if necessary
h_originalaxes  = gca;
if exist('h_axes')
    axes(h_axes)
end

% add frame vectors
h_axes     = [];
for i=1:3
    v      = axis_size*[i==1 i==2 i==3]';
    h_axis = draw_line( frame_origin-v, frame_origin+v, axis_colour, axis_labels{i});
    h_axes = [h_axes h_axis];
end

% format axes 
axis(1.5*axis_size*[-1 1 -1 1 -1 1]);
set(gca,'Position',[0 0 1 1],'PlotBoxAspectRatioMode','manual', ...
        'DataAspectRatioMode','manual', 'CameraViewAngleMode','manual','Visible','off', ...
        'CameraViewAngle',6.5, 'PlotBoxAspectRatio',[1 1 1],'DataAspectRatio',[1 1 1]);
axis fill;

% set current axes back to original at start of function
axes(h_originalaxes);


return



% =========================================================================
% Support functions
% =========================================================================
function h = draw_line(vec_start, vec_end, colour, label)
h(1) = line([vec_start(1) vec_end(1)],[vec_start(2) vec_end(2)],[vec_start(3) vec_end(3)],'Color',colour);
h(2) = text(1.1*vec_end(1), 1.1*vec_end(2), 1.1*vec_end(3), label, 'Color',colour);
return