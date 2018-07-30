function f = sat_model(x,t,Tw_cmd)
% Input
% x         - state variable [q1 q2 q3 q4 w1 w2 w3 h1 h2 h3]
% t         - instantaenous time 
% Tw_cmd    - Wheel command
global CONST

J      = CONST.J;
D_I_RW = CONST.D_I_RW;

% Pre-Allocate Space
f    = zeros(10,1);

% Quaternion, Angular Velocity and Wheel Speed
q  = [x(1);x(2);x(3);x(4)];
w  = [x(5);x(6);x(7)];
wh = [x(8);x(9);x(10)];

% Cross Product Matrices 
wc = [  0   -w(3)  w(2);
       w(3)   0   -w(1);
      -w(2)  w(1)   0  ];

% Integration Functions
om   = [-wc w;
        -w' 0];

T_cmd = D_I_RW*Tw_cmd;

qdot = 0.5*om*q;                      % Satellite Kinetics
wdot = inv(J)*(-wc*J*w + T_cmd);        % Satellite Dynamics
hdot = -wc*wh - T_cmd;                  % Wheel Angular Momentum in Body Frame

f(1:4,:)  = qdot; % Quaternion Rate
f(5:7,:)  = wdot; % Angular Acceleration
f(8:10,:) = hdot; % Angular Momentum Rate
