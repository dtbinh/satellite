function R = eul2R(eul,type)
% Input Euler Angles transform from Coordinate Frame A to Frame B
% Output Transformation Matrix transform vector in Frame A to Frame B
x = eul(1);
y = eul(2);
z = eul(3);

switch type
    case 'zyx'

        R = [  cos(y)*cos(z), cos(x)*sin(z) + cos(z)*sin(x)*sin(y), sin(x)*sin(z) - cos(x)*cos(z)*sin(y);
              -cos(y)*sin(z), cos(x)*cos(z) - sin(x)*sin(y)*sin(z), cos(z)*sin(x) + cos(x)*sin(y)*sin(z);
                      sin(y),                       -cos(y)*sin(x),                        cos(x)*cos(y)];
    case 'xyz'
        R = [                        cos(y)*cos(z),                        cos(y)*sin(z),       -sin(y);
              cos(z)*sin(x)*sin(y) - cos(x)*sin(z), cos(x)*cos(z) + sin(x)*sin(y)*sin(z), cos(y)*sin(x);
              sin(x)*sin(z) + cos(x)*cos(z)*sin(y), cos(x)*sin(y)*sin(z) - cos(z)*sin(x), cos(x)*cos(y)];
    otherwise
        
end

end