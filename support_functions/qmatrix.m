% function Q = qmatrix(q,type)
% ------------------------------------------------------------------------
% This function outputs 4x4 quaternion matrix for quaternion
% multiplication purposes.
% 
% Input 
%   q   -   quaternion input
%  type -   either xyzw or wxyz
% 
% Output
%   Q   -   4x4 matrix 
%
% Revision
%   rusty   -   initial     03 Aug 2018
% 
% Reference
%   rusty
% -------------------------------------------------------------------------
function Q = qmatrix(q,type)

    switch type
        case 'wxyz'

            Q = [q(1) -q(2) -q(3) -q(4);
                 q(2)  q(1)  q(4) -q(3);
                 q(3) -q(4)  q(1)  q(2);
                 q(4)  q(3) -q(2)  q(1)];
        case 'xyzw'
            Q = [ q(4)  q(3) -q(2)  q(1);
                 -q(3)  q(4)  q(1)  q(2);
                  q(2) -q(1)  q(4)  q(3);
                 -q(1) -q(2) -q(3)  q(4)];
    end

end