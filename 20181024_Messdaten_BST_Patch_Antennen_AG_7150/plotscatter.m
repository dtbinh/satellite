function plotscatter(x,y,z,size)
if ~exist('size','var')
    size = 200;
end
k = 1;
for i=1:length(x(:,1))
    for j = 1:length(x(1,:))
        
    plot_x(k) = x(i,j);
    plot_y(k) = y(i,j);
    plot_z(k) = z(i,j);
    
    k = k+1;
    end
end

scatter(plot_x,plot_y,size,plot_z,'filled','s');
xlim([-180 180]) 
ylim([0 180])
set(get(colorbar,'title'),'String', 'gain [dBi]', 'FontSize', 18, 'fontname', 'arial');
end