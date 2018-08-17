function output = controllerVdot(input)
%% GLOBAL PARAMETERS
global CONST CTRL_VDOT
SSaxis   = CONST.SSaxis;   % [axis] Sun Sensor rotation axis 1 = x, 2 = y, 3 = z
SSangles = CONST.SSangles; % [rad] Sun Sensors fram angles

K_d = CTRL_VDOT.K_d;
persistent v1_old v2_old

%% INPUT
S_I    = input(1:3,1);    % Measured Sun vector in Sensor Frame 
R_B_I  = input(1:3,2:4);  %  
B_B    = input(1:3,5); %
% S_S_m  = -input(1:3,5:8); %
 
%% SUN POINTING CONTROL

S_B = R_B_I*S_I; % True Body Frame

v1_cur = B_B;
v2_cur = S_B;

if isempty(v1_old)
    v1_old = v1_cur;
end

if isempty(v2_old)
    v2_old = v2_cur;
end

angle1     = vangle(v1_cur,v1_old);
angle2     = vangle(v2_cur,v2_old);

if angle1 < 1e-16
    tau_tgt  = [0;0;0];
else
    tau_tgt   = - K_d*angle1*vnorm(cross(v1_cur,v1_old)) - K_d*angle2*vnorm(cross(v2_cur,v2_old));
end

v1_old = v1_cur;
v2_old = v2_cur;

output(1:3,1) = tau_tgt;
output(4:6,1) = v1_cur;
end