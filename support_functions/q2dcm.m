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
            % Coordinate Transformation (Verified) - [James Diebel] Representing Attitude: Euler Angles, Unit qernions, and Rotation
            % Output matrix describe vector rotation 
            R = [q(1)^2+q(2)^2-q(3)^2-q(4)^2    2*(q(2)*q(3)+q(1)*q(4))       2*(q(2)*q(4)-q(1)*q(3));
                 2*(q(2)*q(3)-q(1)*q(4))      q(1)^2-q(2)^2+q(3)^2-q(4)^2     2*(q(3)*q(4)+q(1)*q(2));
                 2*(q(2)*q(4)+q(1)*q(3))        2*(q(3)*q(4)-q(1)*q(2))     q(1)^2-q(2)^2-q(3)^2+q(4)^2]; 
            case 'rot_att'
                eta = q(1);     % Euler Angle Component of Quaternion
                eps = q(2:4);   % Euler Vector Component of Quaternion
                R = eye(3) + 2*eta*smtrx(eps) + 2*(smtrx(eps))^2;
                
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
                 % Coordinate Transformation(Verified) Cranfield  Eqion 3-23
                 % Output matrix describe vector rotation
                R = [q(4)^2+q(1)^2-q(2)^2-q(3)^2    2*(q(1)*q(2)+q(3)*q(4))       2*(q(1)*q(3)-q(2)*q(4));
                     2*(q(1)*q(2)-q(3)*q(4))      q(4)^2-q(1)^2+q(2)^2-q(3)^2     2*(q(2)*q(3)+q(1)*q(4));
                     2*(q(1)*q(3)+q(2)*q(4))        2*(q(2)*q(3)-q(1)*q(4))     q(4)^2-q(1)^2-q(2)^2+q(3)^2];
            case 'tsf_att'
                xi  = [ q(4)*eye(3) + smtrx(q(1:3,1)) ; -q(1:3,1)' ]; % [John_Crassidis] Eqion (A.174a)
                psi = [ q(4)*eye(3) - smtrx(q(1:3,1)) ; -q(1:3,1)' ]; % [John_Crassidis] Eqion (A.174b)

                R = xi' * psi;
        end
end

        
        
        