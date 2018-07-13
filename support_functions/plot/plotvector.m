function [plot,label]= plotvector(V, R, color, name, factor)
% Default Setting
if ~exist('color','var')
    color = [1 1 1];
end

if ~exist('name','var')
    name = '';
end

if ~exist('factor','var')
    factor = 1.0;
end



% Normalize Vector
V = V./norm(V)*factor;

% Set Label
if exist('name','var')
   label = text(V(1)+R(1), V(2)+R(2),V(3)+R(3),name, 'Color',color,'FontSize',7.5);
end

% Plot
plot = line([V(1)+R(1) R(1)],[V(2)+R(2) R(2)],[V(3)+R(3) R(3)],'Color',color);
hold on;
        
end
