function [] = Vis3DMoveSpacecraft(h_spacecraft,r)
% Vis3DMoveSpacecraft. Move spacecraft object by position 'r' 

r = reshape(r,3,1);

nhandles = max(size(h_spacecraft));
for i=1:nhandles
    try
        set(h_spacecraft(i),'Xdata',get(h_spacecraft(i),'Xdata') + r(1) );
        set(h_spacecraft(i),'Ydata',get(h_spacecraft(i),'Ydata') + r(2) );
        set(h_spacecraft(i),'Zdata',get(h_spacecraft(i),'Zdata') + r(3) );
    catch
        set(h_spacecraft(i),'Position',get(h_spacecraft(i),'Position') + r');
    end
end
    

