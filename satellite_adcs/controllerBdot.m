function output=controllerBdot(input)
%% GLOBAL PARAMETERS
global CONST CTRL_BDOT

K_d = CTRL_BDOT.K_d;

w_O = CONST.w_O;
%% INPUT
R_O_B  = input(1:3,1:3);
R_B_I  = input(1:3,4:6);
B_B_m  = input(1:3,7);
Bdot_I = input(1:3,8);
w_B_BI = input(1:3,9);

%% ANGULAR VELOCITY
w_O_OI = [0;-w_O;0];             % [rad/s] Angular Velocity of Orbit to Inertia Frame expressed in Orbit Frame
w_B_OI = R_O_B'*w_O_OI;          % [rad/s] Angular Velocity of Orbit to Inertia Frame expressed in Orbit Frame
w_B_BO = w_B_BI - w_B_OI;        % [rad/s] Angular Velocity of Body to Orbit Frame expressed in Body Frame

B_B    = B_B_m;                  % [T] Magnetic Vector in Body Frame
B_I    = R_B_I'*B_B;             % [T] Magnetic Vector in Inertia Frame
Bdot_O = R_B_I*Bdot_I-smtrx(w_O_OI)*B_I;     % [T/s] Magnetic Vector Rate in Body Frame

%% MAGNETIC DIPOLE
Bdot_B  = R_O_B'*Bdot_O-smtrx(w_B_BO)*B_B; % Bdot in Body Frame
m_B     = (-K_d/norm(B_B,2)^2) * Bdot_B;     % Magnetic Dipole desired from Bdot Controller 

% Current Scaling 
[m_B,j] = currentScaling(m_B);


tau_m   = cross(m_B,B_B);


%% OUTPUT
output = tau_m;
end