% function R = q2dcm(q,qorder,rtype)
% -------------------------------------------------------------------------
% This function converts quaternion into rotation matrix
% 
% Example
%   q2dcm(q,'xyzw','tsf')
% 
% Input
%   q       - quaternion
%   qorder  - quaternion order 'wxyz' or 'xyzw'
%   rtype   - dcm type
%
% Output
%   R       - dcm matrix for rotation
% 
% Revision
%   rusty   - initiate      01 may 2018
%
% ------------------------------------------------------------------------- 

function R = q2dcm(q,qorder,rtype)
if ~exist('qorder','var')
    qorder = 'xyzw'; 
end

if ~exist('rtype','var')
    rtype = 'tsf'; 
end

switch qorder
    case 'wxyz' % q = [ eta eps1 eps2 eps3] 
        switch rtype
            case 'rot'
            % Vector Rotation (Verified) Wiki Dipl. -Ing. Karsten Gro�ekatth�fer
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
        switch rtype % q = [eps1 eps2 eps3 eta] 
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

        
        
        