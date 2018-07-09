
function updateVector(plot, V, R, factor)
if ~exist('factor','var')
    factor = 1.0;
end
% Normalize Vector
V = V./norm(V)*factor;
% Update Plot
set(plot,'XData',[V(1)+R(1) R(1)]);
set(plot,'YData',[V(2)+R(2) R(2)]);
set(plot,'ZData',[V(3)+R(3) R(3)]);
end