
function [plot,label]= plotposition(R ,color,marker,name)

if ~exist('name','var')
    label = '';
else
    label = text(R(1),R(2),R(3),name, 'Color',color,'FontSize',7.5);
end

% plot Position
plot =  line(R(1),R(2),R(3),'Color',color,'Marker',marker);


end
