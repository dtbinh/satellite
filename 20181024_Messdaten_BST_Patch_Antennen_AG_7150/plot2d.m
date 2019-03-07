function plot2d(azi, ele, mag, plane, name)


fig = figure;
splot = surf(azi,ele,mag);
set(get(colorbar,'title'),'String', 'gain [dBi]', 'FontSize', 18, 'fontname', 'arial');
caxis([-20 30])
set(splot,'edgecolor','none');
set(fig,'Name',name);
xlabel('Azimuth: \Phi [deg]');
ylabel('Elevation: \Theta [deg]');
view(plane);
end