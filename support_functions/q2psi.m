% function qout = q2psi( qin )
% ------------------------------------------------------------------------
% This function output the xi matrix of the input qinernion 
%
%  qout =  [ qin(4)*eye(3) + smtrx(qin(1:3,1)) ; 
%                    -qin(1:3,1)'              ];
% 
% Input
%   qin    -   qinernion [xyzw]
% 
% Output 
%   xiq     -   xi matrix 
% 
% Revision
%   rusty   -   initial         12 jul 2018
%
% Reference
%   crassidis  

function qout = q2psi( qin )

qout = [ qin(4)*eye(3) - smtrx(qin(1:3,1)) ; 
                -qin(1:3,1)' ]; 
return