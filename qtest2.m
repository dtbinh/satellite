close all
clear all
clc
format long

%   q1 = [-0.406;-0.579;0.579;0.406]; % ST 2 BODY
  q1 = [-0.406;-0.579;0.579;-0.406]; % BODY 2 ST
  
%   q2 = [0.348604914730763;0.479103498052907;0.705332315026185;0.389401705384719]; % ST
 q2 = [-0.668651806096415;0.520014771048744;-0.504458171075601;-0.167510406875252]; % BODY


q3 = qmul(q1,q2,'*')
% BODY FRAME
% qsat2iner = qmul(qst2sat,qst2iner,'*')
%    qout 	- q1.q2   case '.' - q1 is executed then q2
%    qout   - q1*q2   case '*' - q2 is executed then q1
%   0.220182623875386  -0.080945347119646	 0.400725124211092  0.885685510944004	
%  -0.220182623875386	0.080945347119646	-0.400725124211092 -0.885685510944004	


% CLYDE SPACE quaternion is wxyz
% BST quaternion is xyzw

% DATA 1
% BODY FRAME
% -0.669121794011011	0.519074795219551	-0.504424600510273
% -0.168618235531086
%
% ST Frame
% 0.348604914730763
%    0.479103498052907
%    0.705332315026185
%    0.389401705384719
%  E321:    117.195   -6.813   72.535 deg




% E321:    -95.199  -58.222 -145.541 deg
% 

% DATA 2
% 	BODY FRAME
% -0.668651806096415	0.520014771048744	-0.504458171075601
% -0.167510406875252


% ST FRAME
% 0.348489129850947
%    0.477821941721499
%    0.706096884651538
%    0.389706425406204
% E321:    117.178   -6.876   72.374 deg