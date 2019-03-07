function [x,y,z] = bst_sph2cart(azi,ele,mag)
    
    x = mag .* sin(ele) .* cos(azi);
    y = mag .* sin(ele) .* sin(azi);
    z = mag .* cos(ele);

end