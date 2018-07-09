function eul = q2eul(q,type,option)
switch type
    case 'wxyz'
        switch option
            case 'zyx'
                % wxyz, %321, James Diebel
                eul(1) = atan2(-2*q(2)*q(3)+2*q(1)*q(4),q(2)^2+q(1)^2-q(4)^2-q(3)^2);
                eul(2) = asin(2*q(2)*q(4)+2*q(1)*q(3));
                eul(3) = atan2(-2*q(3)*q(4)+2*q(1)*q(2),q(4)^2-q(3)^2-q(2)^2+q(1)^2);

            case 'xyz'
                eul(1) = atan2(2*q(3)*q(4)+2*q(1)*q(2),q(4)^2-q(3)^2-q(2)^2+q(1)^2);
                eul(2) = -asin(2*q(2)*q(4)-2*q(1)*q(3));
                eul(3) = atan2(2*q(2)*q(3)+2*q(1)*q(4),q(2)^2+q(1)^2-q(4)^2-q(3)^2);
                
            otherwise
        end
    case 'xyzw'
        q = qinvert(q,'wxyz');
        
        switch option
            case 'zyx'
                % wxyz, %321, James Diebel
                eul(1) = atan2(-2*q(2)*q(3)+2*q(1)*q(4),q(2)^2+q(1)^2-q(4)^2-q(3)^2);
                eul(2) = asin(2*q(2)*q(4)+2*q(1)*q(3));
                eul(3) = atan2(-2*q(3)*q(4)+2*q(1)*q(2),q(4)^2-q(3)^2-q(2)^2+q(1)^2);

            case 'xyz'
                eul(1) = atan2(2*q(3)*q(4)+2*q(1)*q(2),q(4)^2-q(3)^2-q(2)^2+q(1)^2);
                eul(2) = -asin(2*q(2)*q(4)-2*q(1)*q(3));
                eul(3) = atan2(2*q(2)*q(3)+2*q(1)*q(4),q(2)^2+q(1)^2-q(4)^2-q(3)^2);
                
            otherwise
        end
            
    otherwise
end
    
end