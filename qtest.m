close all
clear all
clc
format

% q1 = [-0.405579787672639;-0.579227965339569;0.579227965339569;0.405579787672639];
 q1 = [-0.406;-0.579;0.579;0.406];
% q2 = [0.407855512286827;0.705659997314355;-0.414421914865046;0.404867731972606];
 q2 = [0.0700013428226129;0.398019336645629;0.727648717604404;0.554223177118303];


% eul = q2eul(q1)
% eul*rad2deg
% 
% q1  = eul2q(eul)
qmul(q2,q1,'.')