function [xq,yq,vq] = plotinterpolate(azi,ele,mag)

k=1;
for i=1:length(azi(:,1))
    for j=1:length(ele(1,:))
        x(k) = azi(i,j);
        y(k) = ele(i,j);
        z(k) = mag(i,j);
        k=k+1;
    end
end
min_azi = min(min(azi));
max_azi = max(max(azi));
stp_azi = (max_azi-min_azi)/length(azi(1,:));

min_ele = min(min(ele));
max_ele = max(max(ele));
stp_ele = (max_ele-min_ele)/length(ele(:,1));

grid_azi = min_azi:stp_azi:max_azi;
grid_ele = min_ele:stp_ele:max_ele;

[xq,yq] = meshgrid(grid_azi,grid_ele );

% method can be 'linear', 'nearest', 'natural', 'cubic', or 'v4'. The default method is 'linear'.
vq = griddata(x,y,z,xq,yq);
end