% ------------------------------------------------------------------------
% Input
% x     - state variable [q1 q2 q3 q4 w1 w2 w3 wh1 wh2 wh3]
% t     - instantaenous time 
% sat   - satellite model
% ------------------------------------------------------------------------
function xd = satellite_model(x,t,sat,ctrl)

% Pre-Allocate Space
xd  = zeros(10,1);

% Quaternion, Angular Velocity and Wheel Speed
q  = [x(1);x(2);x(3);x(4)];
w  = [x(5);x(6);x(7)];
hw = [x(8);x(9);x(10)];

% Integration Functions
om   = [-smtrx(w) w;
           -w'    0];
% Satellite Kinetics
qdot = 0.5*om*q; 

% Satellite Dynamics
wdot = inv(sat.I)*(-smtrx(w)*sat.I*w + ctrl.trq);

% RW Angular Momentum
hdot = -smtrx(w)*hw-ctrl.trq;                   

xd(1:4,:)  = qdot; % Quaternion Rate
xd(5:7,:)  = wdot; % Angular Acceleration
xd(8:10,:) = hdot; % Wheel Angular Momentum Rate
