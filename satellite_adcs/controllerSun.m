function output = controllerSun(input)
%% INPUT
S_I    = input(1:3,1);    % [-] Measured Sun vector in Inertial Frame 
w_B_BI = input(1:3,2);    % [rad/s2] Angular Velocity in Body Frame
R_B_I  = input(1:3,3:5);  % [-]  
S_B_m  = -input(1:3,6); % [-] Measured Sun Sensor in Sensor Frame 

%% CONSTANT PARAMETERS
global CONST CTRL_SP
I   = CONST.I;

K_p = CTRL_SP.K_p;
K_v = CTRL_SP.K_v;
w_B_tgt = CTRL_SP.w_B_tgt;     % Desired Angular Velocity in Body Frame
S_B_tgt = CTRL_SP.S_B_tgt;     % Desired Sun Vector in Body Frame (should be the optimal sun vector)
type    =  CTRL_SP.type;

%% SUN POINTING CONTROL
S_B = R_B_I*S_I;
S_B_m = S_B;

switch type
     case 'full'
        % Angular Momentum control
        torq   = K_p * cross(S_B_tgt,S_B_m) - K_v*(w_B_BI - w_B_tgt) + cross(w_B_tgt,I*w_B_BI);
    case 'angmom'
        % Angular Momentum control
        torq   = K_p * cross(S_B_tgt,S_B_m) - K_v*(w_B_BI - w_B_tgt);
    case 'noangmom'
        % Zero-momentum control
        torq   = K_p * cross(S_B_tgt,S_B_m) - K_v*w_B_BI;
    case 'modified'
        % Modified
        
        angle  = vangle(S_B_tgt,S_B);     % Angle between Desired Vector and Actual Sun Vector
        
        torq   = K_p * angle * cross(S_B_tgt,S_B_m) - K_v*(w_B_BI - w_B_tgt);
        
    case 'rusty'
       if norm(w_B_BI)>dps2rps(0.3)
            torq   = K_p * cross(S_B_tgt,S_B_m)- K_v*(w_B_BI - w_B_tgt);
       else
           torq   = K_p * cross(S_B_tgt,S_B_m);
       end
end
        

output = torq ;    % [Nm] 

end