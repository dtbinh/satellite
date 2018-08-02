function output = controllerSun(input)
%% INPUT
S_I    = input(1:3,1);   % Measured Sun vector in Sensor Frame 
w_B_BI = input(1:3,2);   % [rad/s2] Angular Velocity Desired
R_B_I  = input(1:3,3:5); %  
S_S_m  = -input(1:3,6:9); %

%% CONSTANT PARAMETERS
global CONST CTRL_SP
SSaxis   = CONST.SSaxis;   % [axis] Sun Sensor rotation axis 1 = x, 2 = y, 3 = z
SSangles = CONST.SSangles; % [rad] Sun Sensors fram angles
I   = CONST.I;

K_p = CTRL_SP.K_p;
K_v = CTRL_SP.K_v;
w_tgt = CTRL_SP.w_tgt;       % Desired Angular Velocity
S_tgt = CTRL_SP.S_tgt;       % Desired Sun Vector in Body Frame (should be the optimal sun vector)

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
angle  = acosd(dot(S_tgt,S_B)/(norm(S_tgt)*norm(S_B)));        % Angle between Desired Vector and Actual Sun Vector

angle  = acos(dot(S_tgt,S_B_m_f)/(norm(S_tgt)*norm(S_B_m_f))); % Angle between Desired Vector and Measured Vector

torq   = K_p * cross(S_tgt,S_B_m_f) - K_v*(w_B_BI - w_tgt) + cross(w_tgt,I*w_B_BI);
output = [torq ;angle];    % [Nm] 

end