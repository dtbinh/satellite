function output = controllerSun(input)
%% INPUT
S_I    = input(1:3,1);   % Measured Sun vector in Sensor Frame 
w_B_BI = input(1:3,2);   % [rad/s2] Angular Velocity Desired
R_B_I  = input(1:3,3:5); %  
S_S_m  = -input(1:3,6:9); %

%% CONSTANT PARAMETERS
global CONST
SSaxis   = CONST.SSaxis;   % [axis] Sun Sensor rotation axis 1 = x, 2 = y, 3 = z
SSangles = CONST.SSangles; % [rad] Sun Sensors fram angles

kp = 5e-06;
kv = 5e-04;

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


S_B_d = [1;0;0];              % Desired Sun Vector in Body Frame (should be the optimal sun vector)

S_B = R_B_I*S_I;
angle  = acosd(dot(S_B_d,S_B)/(norm(S_B_d)*norm(S_B)));  % Angle between Desired Vector and Measured Vector

da  = acos(dot(S_B_d,S_B_m_f)/(norm(S_B_d)*norm(S_B_m_f)));

torq   = kp * cross(S_B_d,S_B_m_f) - kv*w_B_BI; 
output = [torq ;angle];    % [Nm] 

end