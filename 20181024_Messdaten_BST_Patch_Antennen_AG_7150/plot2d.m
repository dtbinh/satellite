function plot2d(azi, ele, mag, plane, name, cscale)

if ~exist('cscale','var')
    cscale = [-20 30];
end

fig = figure;
splot = surf(azi,ele,mag);
set(get(colorbar,'title'),'String', 'gain [dBi]', 'FontSize', 10, 'fontname', 'arial');
caxis(cscale);
set(splot,'edgecolor','none');
set(fig,'Name',name);
xlabel('Azimuth: \Phi [deg]');
ylabel('Elevation: \Theta [deg]');
view(plane);
end