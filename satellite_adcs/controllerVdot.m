function output = controllerVdot(input)
%% GLOBAL PARAMETERS
global CONST CTRL_VDOT
SSaxis   = CONST.SSaxis;   % [axis] Sun Sensor rotation axis 1 = x, 2 = y, 3 = z
SSangles = CONST.SSangles; % [rad] Sun Sensors fram angles

K_d = CTRL_VDOT.K_d;
persistent v_old

%% INPUT
S_I    = input(1:3,1);    % Measured Sun vector in Sensor Frame 
R_B_I  = input(1:3,2:4);  %  
B_B    = input(1:3,5); %
% S_S_m  = -input(1:3,5:8); %
 
%% SUN POINTING CONTROL
% R_S_B(:,:,1) = dcm(SSaxis(1),SSangles(1));
% R_S_B(:,:,2) = dcm(SSaxis(2),SSangles(2));
% R_S_B(:,:,3) = dcm(SSaxis(3),SSangles(3));
% R_S_B(:,:,4) = dcm(SSaxis(4),SSangles(4));
% 
% S_B_m(1:3,1) = R_S_B(:,:,1)'*S_S_m(1:3,1);  % Measured Sun Vector in Body Frame
% S_B_m(1:3,2) = R_S_B(:,:,2)'*S_S_m(1:3,2);  % Measured Sun Vector in Body Frame
% S_B_m(1:3,3) = R_S_B(:,:,3)'*S_S_m(1:3,3);  % Measured Sun Vector in Body Frame
% S_B_m(1:3,4) = R_S_B(:,:,4)'*S_S_m(1:3,4);  % Measured Sun Vector in Body Frame
% 
% if (S_B_m(1:3,1) ~= 0)
%     S_B_m_f = S_B_m(1:3,1);
% else if  (S_B_m(1:3,2) ~= 0)
%         S_B_m_f = S_B_m(1:3,2);
%     else if  (S_B_m(1:3,3) ~= 0)
%             S_B_m_f = S_B_m(1:3,3);
%         else if  (S_B_m(1:3,4) ~= 0)
%                 S_B_m_f = S_B_m(1:3,4);
%             end
%         end
%     end
% end


S_B = R_B_I*S_I; % True Body Frame

v_cur = B_B;

if isempty(v_old)
    v_old = v_cur;
end

angle     = vangle(v_cur,v_old);
if angle < 1e-16
    tau_tgt  = [0;0;0];
else
    
    tau_tgt   = -K_d * angle * vnorm(cross(v_cur,v_old));
end

v_old = v_cur;

output(1:3,1) = tau_tgt;
output(4:6,1) = v_cur;
end