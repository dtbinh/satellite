function eul = dcm2eul(R,etype)

switch etype
    case 'xyz'
        if (R(1,3)<1)
            if (R(1,3)>-1)
                thetaY = asin(-R(1,3));
                thetaX = atan2(R(2,3),R(3,3));
                thetaZ = atan2(R(1,2),R(1,1));
            else
                thetaY = pi/2;
                thetaX = atan2(R(2,1),R(2,2));
                thetaZ = 0;
            end

        else
                thetaY = -pi/2;
                thetaX = -atan2(R(2,1),R(2,2));
                thetaZ = 0;
        end
    case 'zyx'
        if (R(3,1)<1)
            if (R(3,1)>-0.99999)
                thetaY = asin(R(3,1));
                thetaZ = atan2(-R(2,1),R(1,1));
                thetaX = atan2(-R(3,2),R(3,3));
            else % (R(3,1)=-1) thetaY = pi/2;
                thetaY = -pi/2;
                thetaZ = atan2(-R(2,3),R(2,2));
                thetaX = 0;
            end

        else     % (R(3,1)=+1), thetaY = -pi/2;
                thetaY = pi/2;
                thetaZ = -atan2(-R(2,3),R(2,2));
                thetaX = 0;
        end
    otherwise

end
eul = [thetaX; thetaY; thetaZ];
end