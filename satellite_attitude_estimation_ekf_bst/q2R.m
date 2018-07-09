function R = q2R(q,order,type) % example q2R(q,'wxyz','tsf')
switch order
    case 'wxyz' % q = [ eta eps1 eps2 eps3] 
        switch type
            case 'rot'
            % Vector Rotation (Verified) Wiki Dipl. -Ing. Karsten Groÿekatthöfer
            % Output matrix describe frame transformation
            R = [q(1)^2+q(2)^2-q(3)^2-q(4)^2    2*(q(2)*q(3)-q(1)*q(4))       2*(q(2)*q(4)+q(1)*q(3));
                 2*(q(2)*q(3)+q(1)*q(4))      q(1)^2-q(2)^2+q(3)^2-q(4)^2     2*(q(3)*q(4)-q(1)*q(2));
                 2*(q(2)*q(4)-q(1)*q(3))        2*(q(3)*q(4)+q(1)*q(2))     q(1)^2-q(2)^2-q(3)^2+q(4)^2]; 
 
            case 'tsf'
            % Coordinate Transformation (Verified) - [James Diebel] Representing Attitude: Euler Angles, Unit Quaternions, and Rotation
            % Output matrix describe vector rotation 
            R = [q(1)^2+q(2)^2-q(3)^2-q(4)^2    2*(q(2)*q(3)+q(1)*q(4))       2*(q(2)*q(4)-q(1)*q(3));
                 2*(q(2)*q(3)-q(1)*q(4))      q(1)^2-q(2)^2+q(3)^2-q(4)^2     2*(q(3)*q(4)+q(1)*q(2));
                 2*(q(2)*q(4)+q(1)*q(3))        2*(q(3)*q(4)-q(1)*q(2))     q(1)^2-q(2)^2-q(3)^2+q(4)^2]; 
            
         end

    case 'xyzw'
        switch type % q = [eps1 eps2 eps3 eta] 
            case 'rot' %  Vector Rotation 
                 %  Vector Rotation (Verified) Rusty
                 % Output matrix describe frame transformation 
                R = [q(4)^2+q(1)^2-q(2)^2-q(3)^2    2*(q(1)*q(2)-q(4)*q(3))       2*(q(1)*q(3)+q(2)*q(4));
                     2*(q(1)*q(2)+q(3)*q(4))      q(4)^2-q(1)^2+q(2)^2-q(3)^2     2*(q(2)*q(3)-q(1)*q(4));
                     2*(q(1)*q(3)-q(2)*q(4))        2*(q(2)*q(3)+q(4)*q(1))     q(4)^2-q(1)^2-q(2)^2+q(3)^2];
                
            case 'tsf'     
                 % Coordinate Transformation(Verified) Cranfield  Equation 3-23
                 % Output matrix describe vector rotation
                R = [q(4)^2+q(1)^2-q(2)^2-q(3)^2    2*(q(1)*q(2)+q(3)*q(4))       2*(q(1)*q(3)-q(2)*q(4));
                     2*(q(1)*q(2)-q(3)*q(4))      q(4)^2-q(1)^2+q(2)^2-q(3)^2     2*(q(2)*q(3)+q(1)*q(4));
                     2*(q(1)*q(3)+q(2)*q(4))        2*(q(2)*q(3)-q(1)*q(4))     q(4)^2-q(1)^2-q(2)^2+q(3)^2];
            

        end
end

        
        
        