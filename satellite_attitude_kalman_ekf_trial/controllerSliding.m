function output = controllerSliding(input)
%% GLOBAL PARAMETERS
global CONST

I = CONST.I; % [kg/m2] Satellite's Moment of Inertia

%% INPUT
q_B_I       = input(1:4,1);   % [xyzw]  Quaternion Actual   
q_B_O     = input(5:8,1);   % [xyzw]  Quaternion Desired  
w_B_BI      = input(9:11,1);  % [rad/s] Angular Velocity Actual
w_O_OI    = input(12:14,1); % [rad/s] Angular Velocity Desired
wdot_B_BI_d = input(15:17,1); % [rad/s2] Angular Acceleration Desired

%% SLIDING MODE CONTROL
% Parameter
k   = 0.05;             % Proportional gain
eps = 0.001;
g   = 0.005*eye(3);

q_O_I = qmul(qinvert(q_B_O),q_B_I);
q_B_I_d = q_O_I;

R_B_O = q2dcm(q_B_O);
w_B_BI_d = R_B_O*w_O_OI; 

% Sliding Control
dq_B_I    = [q2xi(q_B_I_d)'*q_B_I;
               q_B_I_d'*q_B_I    ];         

% Sliding Surface
ss = (w_B_BI-w_B_BI_d) + k*sign(dq_B_I(4,1))*dq_B_I(1:3,1);

% Sliding Mode Control
usat = [saturation(ss(1),eps);
        saturation(ss(2),eps);
        saturation(ss(3),eps)];

dqdot_B_I = q2xi(q_B_I_d)'*q2xi(q_B_I)*w_B_BI - q2xi(q_B_I)'*q2xi(q_B_I_d)*w_B_BI_d;
torq      = smtrx(w_B_BI)*(I*w_B_BI) - I*(0.5*k*sign(dq_B_I(4,1))*dqdot_B_I - wdot_B_BI_d + g*usat);

output = [torq, ss, g*usat];    % [Nm] 

end