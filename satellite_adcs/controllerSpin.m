function output = controllerSpin(input)
%% INPUT
S_I    = input(1:3,1);    % [-] Measured Sun vector in Inertial Frame 
w_B_BI = input(1:3,2);    % [rad/s2] Angular Velocity in Body Frame
R_B_I  = input(1:3,3:5);  % [-]  
B_B_m    = input(1:3,6);    % [T] Magnetic Field

%% CONSTANT PARAMETERS
global CONST CTRL_SC
I   = CONST.I;
K   = CTRL_SC.K ;
K_1 = CTRL_SC.K_1;
K_2 = CTRL_SC.K_2;
w_tgt = CTRL_SC.w_tgt;       % Desired Angular Velocity
vec_tgt = CTRL_SC.vec_tgt;       % Desired Sun Vector in Body Frame (should be the optimal sun vector)

a_tgt_i = vnorm(S_I);  % [-] unit vector of desired in inertial

h_tgt_a = I*(w_tgt*vec_tgt);     % target angular momentum vector in desired body frame components
h_tgt_i = a_tgt_i*norm(h_tgt_a); % target angular momentum of the satellite in inertial frame components
h_sat_b = I*w_B_BI;              % angular momentum of the satellite in body frame components
h_tgt_b = R_B_I*h_tgt_i;         % h target vector in body frame components

dh   = h_sat_b - h_tgt_b;  % h error vector in body frame components
e_ha = h_sat_b - h_tgt_a;  % h error vector in body frame components
P = diag([1 0 1]);

type = CTRL_SC.type;

switch type
    case 'ruiter'
        % Only for y-axis
        m_B = -(K/norm(B_B_m,2)^2)*cross(B_B_m,( dh  +  K_1*[0;e_ha(2);0]  +  K_2*P*w_B_BI ));
    case 'buhl'
    
        m_B = -(K/norm(B_B_m,2)^2)*cross(B_B_m,( dh  +        K_1*e_ha  ));
        
    case 'hihb'
        d_w_B = w_B_BI - w_tgt*vec_tgt;
  
        m_B = -(K/norm(B_B_m,2)^2)*cross(B_B_m,( dh  +        K_1*e_ha     +  K_2*d_w_B ));

            
end

torq  = cross(m_B,B_B_m);

output = torq ;    % [Nm] 

end