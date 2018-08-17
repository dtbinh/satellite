function output = controllerSun(input)
%% INPUT
S_I    = input(1:3,1);    % [-] Measured Sun vector in Inertial Frame 
w_B_BI = input(1:3,2);    % [rad/s2] Angular Velocity in Body Frame
R_B_I  = input(1:3,3:5);  % [-]  
S_S_m  = -input(1:3,6:9); % [-] Measured Sun Sensor in Sensor Frame 

%% CONSTANT PARAMETERS
global CONST CTRL_SP
SSaxis   = CONST.SSaxis;   % [axis] Sun Sensor rotation axis 1 = x, 2 = y, 3 = z
SSangles = CONST.SSangles; % [rad] Sun Sensors fram angles
I   = CONST.I;

K_p = CTRL_SP.K_p;
K_v = CTRL_SP.K_v;
w_B_tgt = CTRL_SP.w_B_tgt;     % Desired Angular Velocity in Body Frame
S_B_tgt = CTRL_SP.S_B_tgt;     % Desired Sun Vector in Body Frame (should be the optimal sun vector)
type    =  CTRL_SP.type;
%% SUN POINTING CONTROL
R_S_B(:,:,1) = dcm(SSaxis(1),SSangles(1));
R_S_B(:,:,2) = dcm(SSaxis(2),SSangles(2));
R_S_B(:,:,3) = dcm(SSaxis(3),SSangles(3));
R_S_B(:,:,4) = dcm(SSaxis(4),SSangles(4));

S_B_m(1:3,1) = R_S_B(:,:,1)'*S_S_m(1:3,1);  % Measured Sun Vector in Body Frame
S_B_m(1:3,2) = R_S_B(:,:,2)'*S_S_m(1:3,2);  % Measured Sun Vector in Body Frame
S_B_m(1:3,3) = R_S_B(:,:,3)'*S_S_m(1:3,3);  % Measured Sun Vector in Body Frame
S_B_m(1:3,4) = R_S_B(:,:,4)'*S_S_m(1:3,4);  % Measured Sun Vector in Body Frame

if (S_B_m(1:3,1) ~= 0)
    S_B_m_f = S_B_m(1:3,1);
else if  (S_B_m(1:3,2) ~= 0)
        S_B_m_f = S_B_m(1:3,2);
    else if  (S_B_m(1:3,3) ~= 0)
            S_B_m_f = S_B_m(1:3,3);
        else if  (S_B_m(1:3,4) ~= 0)
                S_B_m_f = S_B_m(1:3,4);
            end
        end
    end
end
S_B = R_B_I*S_I;
S_B_m = S_B;

switch type
     case 0
        % Angular Momentum control
        torq   = K_p * cross(S_B_tgt,S_B_m) - K_v*(w_B_BI - w_B_tgt) + cross(w_B_tgt,I*w_B_BI);
    case 1
        % Angular Momentum control
        torq   = K_p * cross(S_B_tgt,S_B_m) - K_v*(w_B_BI - w_B_tgt);
    case 2
        % Zero-momentum control
        torq   = K_p * cross(S_B_tgt,S_B_m) - K_v*w_B_BI;
    case 3
        % Modified
        
        angle  = vangle(S_B_tgt,S_B);     % Angle between Desired Vector and Actual Sun Vector
        
        torq   = K_p * angle * cross(S_B_tgt,S_B_m) - K_v*(w_B_BI - w_B_tgt);
end
        

output = torq ;    % [Nm] 

end