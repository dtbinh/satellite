% function xiq = xi( quat )
% ------------------------------------------------------------------------
% This function output the xi matrix of the input quaternion 
%
%  xiq =  [ quat(4)*eye(3) + smtrx(quat(1:3,1)) ; 
%                    -quat(1:3,1)'              ];
% 
% Input
%   quat    -   Quaternion [xyzw]
% 
% Output 
%   xiq     -   xi matrix 
% 
% Revision
%   rusty   -   initial         12 jul 2018

function xiq = xi( quat )

xiq = [ quat(4)*eye(3) + smtrx(quat(1:3,1)) ; 
            -quat(1:3,1)'                ];
return