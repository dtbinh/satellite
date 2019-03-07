function [azi, ele, mag] = bst_cart2sph(x,y,z)

[azi,ele,mag] = cart2sph(x,y,z);

% convert elev
for i=1:length(ele(:,1))
    for j=1:length(ele(1,:))
        ele(i,j) = pi/2 - ele(i,j);
    
    end
end

% Remove End Coloumn Data
end