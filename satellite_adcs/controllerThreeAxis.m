function output = controllerThreeAxis(input)
%% GLOBAL PARAMETERS
global CONST CTRL_TA

I = CONST.I; % [kg/m2] Satellite's Moment of Inertia

%% INPUT
q_B_I     = input(1:4,1);   % [xyzw]  Quaternion          (Measured)
w_B_BI    = input(5:7,1);   % [rad/s] Angular Velocity    (Measured)
w_B_BI_tgt   = input(8:10,1);
q_B_I_tgt     = input(11:14,1);

%% THREE AXIS CONTROL
% Parameter
k1   = CTRL_TA.k1;             
k2   = CTRL_TA.k2; 
d1   = CTRL_TA.d1; 
d2   = CTRL_TA.d2; 
kp   = CTRL_TA.kp;
kd   = CTRL_TA.kd;


% Define Quaternion Error
dq   = qmul(q_B_I,qinvert(q_B_I_tgt));

% Rotation Axis & Angle
[axis,angle] =q2axis(dq);

% Define Rotation Axis
u = axis;

% Define Angle
phi = 2*atan2((dq(1:3,1))'*u, dq(4,1));

% Define Parallel & Perpendicular Components
q_par = [sin(phi/2)*u;
          cos(phi/2)];
q_per = qmul(dq,qinvert(q_par));

    
type = CTRL_TA.type;

switch type
    case 'eigenaxis'
        torq = -k1*sign(phi)*dq(1:3,1)  - k2*q_per(1:3,1) - d1*(w_B_BI-w_B_BI_tgt) - d2*(eye(3) - u*u')*(w_B_BI-w_B_BI_tgt);
    case 'qfeedback'
        torq = -kp*sign(dq(4,1))*dq(1:3,1) - kd*(w_B_BI-w_B_BI_tgt) + cross(w_B_BI,I*w_B_BI);
    case 'magnetorquer'

        torq = -eps^2*kp*dq(1:3,1) - eps*beta*sat(1/beta*kd*sat(w_B_BI - w_B_tgt));
    otherwise
        fprintf('CTRL_TA not defined!');
end


output = [torq;w_B_BI_tgt];    % [Nm] 
end