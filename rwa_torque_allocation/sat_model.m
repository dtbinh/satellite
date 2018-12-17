function f = sat_model(x,t,Tw_cmd)
% Input
% x         - state variable [q1 q2 q3 q4 w1 w2 w3 h1 h2 h3]
% t         - instantaenous time 
% Tw_cmd    - Wheel command
global CONST

J      = CONST.J;
D_B_RW = CONST.D_B_RW;
D_RW_B = CONST.D_RW_B;

% Pre-Allocate Space
f    = zeros(10,1);


q  = [x(1);x(2);x(3);x(4)]; % Quaternion
w  = [x(5);x(6);x(7)];      % Angular Velocity
wh = [x(8);x(9);x(10)];     % Angular Momentum
w_h = [x(11);x(12);x(13);x(14)];     % Angular Momentum

% Integration Functions
om   = [-smtrx(w) w;
           -w'    0];

T_cmd1 = D_B_RW(:,1)*Tw_cmd(1); % RW1 torque in Body Frame
T_cmd2 = D_B_RW(:,2)*Tw_cmd(2); % RW2 torque in Body Frame
T_cmd3 = D_B_RW(:,3)*Tw_cmd(3); % RW3 torque in Body Frame
T_cmd4 = D_B_RW(:,4)*Tw_cmd(4); % RW4 torque in Body Frame

T_cmd = T_cmd1 + T_cmd2 + T_cmd3 + T_cmd4;

qdot = 0.5*om*q;                        % Satellite Kinetics
wdot = inv(J)*(-smtrx(w)*J*w + T_cmd);  % Satellite Dynamics
hdot = -smtrx(w)*wh -  T_cmd;           % Angular Momentum in Body Frame
w_hdot = D_RW_B*hdot;


f(1:4,:)   = qdot;   % Quaternion Rate
f(5:7,:)   = wdot;   % Angular Acceleration
f(8:10,:)  = hdot;   % Satellite Angular Momentum Rate
f(11:14,:) = w_hdot; 
