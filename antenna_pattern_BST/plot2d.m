function plot2d(azi, ele, mag, plane, cscale)

splot = surf(azi,ele,mag);
set(get(colorbar,'title'),'String', 'gain [dBi]', 'FontSize', 10, 'fontname', 'arial');
set(splot,'edgecolor','none');

xlabel('Azimuth: \Phi [deg] from Z axis');
ylabel('Elevation: \Theta [deg] from X-axis in Z-direction');
view(plane);
if exist('cscale','var')
   caxis(cscale);
end
end