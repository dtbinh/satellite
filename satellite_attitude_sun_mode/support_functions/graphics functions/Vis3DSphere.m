function h_sphere = Vis3DSphere(sphere_radius, h_mainaxes, flag_newaxes)
% Vis3DSphere. add transparent sphere to current figure window.

h_originalaxes  = gca;

% if flag_newaxes=false add sphere to axes specified by h_mainaxes
% otherwise create new axes
if ~flag_newaxes
    h_sphere     = h_mainaxes;
else
    h_sphere     = axes('position', get(h_mainaxes,'Position'));
    setup_fig_axes(1.5*sphere_radius);
end

% add transparent sphere with radius of sphere_radius
[xs, ys, zs] = sphere(20);
ps           = surf(sphere_radius*xs,sphere_radius*ys,sphere_radius*zs,'Parent',h_sphere);
set(ps,'facecolor','none','edgecolor',0.7*[1 1 1],'LineStyle',':');

axes(h_originalaxes);



% =========================================================================
% supporting functions
% =========================================================================
function setup_fig_axes(scale)
set(gca,'Position',[0 0 1 1],'PlotBoxAspectRatioMode','manual','DataAspectRatioMode','manual','CameraViewAngleMode','manual');
set(gca,'Visible','off');
axis(scale*[-1 1 -1 1 -1 1]);
set(gca,'CameraViewAngle',6.5);
set(gca,'PlotBoxAspectRatio',[1 1 1],'DataAspectRatio',[1 1 1]);axis fill;
return
