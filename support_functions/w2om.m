% function omout = w2om( win )
% ------------------------------------------------------------------------
% This function output the xi matrix of the input qinernion 
%
%  omout = [ -smtrx(win) w;
%               -w'     0];
% 
% Input
%   qin    -   qinernion [xyzw]
% 
% Output 
%   qout     -   xi matrix 
% 
% Revision
%   rusty   -   initial         12 jul 2018
% 
% Reference
%   crassidis
% ------------------------------------------------------------------------

function omout = w2om( win )

omout = [ -smtrx(win) win;
              -win'     0];
return