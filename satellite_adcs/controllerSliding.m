function output = controllerSliding(input)
%% GLOBAL PARAMETERS
global CONST CTRL_SM

I = CONST.I; % [kg/m2] Satellite's Moment of Inertia

%% INPUT
q_B_I       = input(1:4,1);   % [xyzw]  Quaternion Actual   
q_B_I_tgt   = input(5:8,1);   % [xyzw]  Quaternion Desired  
w_B_BI      = input(9:11,1);  % [rad/s] Angular Velocity Actual
w_B_BI_tgt  = input(12:14,1); % [rad/s] Angular Velocity Desired
wd_B_BI_tgt = input(15:17,1); % [rad/s2] Angular Acceleration Desired

%% SLIDING MODE CONTROL
% Parameter
k   = CTRL_SM.k;             % Proportional gain
eps = CTRL_SM.eps;
g   = CTRL_SM.g;
G   = CTRL_SM.G;

% Sliding Control
dq_B_I    = [q2xi(q_B_I_tgt)'*q_B_I;
               q_B_I_tgt'*q_B_I    ];         

% Sliding Surface
ss = (w_B_BI-w_B_BI_tgt) + k*sign(dq_B_I(4,1))*dq_B_I(1:3,1);

% Sliding Mode Control
usat = [saturation(ss(1),eps);
        saturation(ss(2),eps);
        saturation(ss(3),eps)];
    
type = CTRL_SM.type;

switch type
    case 'crassidis1'
        dqdot_B_I_img = q2xi(q_B_I_tgt)'*q2xi(q_B_I)*w_B_BI - q2xi(q_B_I)'*q2xi(q_B_I_tgt)*w_B_BI_tgt;
        torq      = smtrx(w_B_BI)*(I*w_B_BI) - I*( 0.5*k*sign(dq_B_I(4,1))*dqdot_B_I_img - wd_B_BI_tgt) - I*G*usat ;
    case 'crassidis2'
        temp = norm(dq_B_I(4,1))*(w_B_BI_tgt - w_B_BI) - cross(sign(dq_B_I(4,1))*dq_B_I(1:3,1),(w_B_BI_tgt + w_B_BI));
        torq      = smtrx(w_B_BI)*(I*w_B_BI) + I*(0.5*k*(temp) + wd_B_BI_tgt - G*usat );
    case 'bst'
        dqd1 = qmul([w_B_BI;0],dq_B_I);
        dqd2 = qmul(dq_B_I,[w_B_BI_tgt;0]);
        dqd  = 0.5*(dqd1 - dqd2);
        dqd_img = dqd(1:3,1);
        torq    = I*wd_B_BI_tgt  - I*k*sign(dq_B_I(4,1))*dqd_img - I*g*ss;
    case 'bst_mod'
        dqd1 = qmul([w_B_BI;0],dq_B_I);
        dqd2 = qmul(dq_B_I,[w_B_BI_tgt;0]);
        dqd  = 0.5*(dqd1 - dqd2);
        dqd_img = dqd(1:3,1);

        torq    = I*wd_B_BI_tgt  - I*k*sign(dq_B_I(4,1))*dqd_img - I*G*usat;
    otherwise
end

output = [torq; ss];    % [Nm] 

end