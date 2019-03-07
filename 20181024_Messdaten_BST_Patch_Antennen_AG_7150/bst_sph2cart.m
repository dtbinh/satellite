function [x,y,z] = bst_sph2cart(azi,ele,mag)

    if (ele > pi)
       fprintf('error'); 
    end
    
    if (azi > pi)
       fprintf('error'); 
    end
    
    x = mag .* sin(ele) .* cos(azi);
    y = mag .* sin(ele) .* sin(azi);
    z = mag .* cos(ele);

end