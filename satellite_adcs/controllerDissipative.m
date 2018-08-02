function output = controllerDissipative(input)
%% GLOBAL PARAMETERS
global CONST
global CTRL_RF

K_d = CTRL_RF.K_d;
K_p = CTRL_RF.K_p;
w_O = CONST.w_O;

%% INPUT
R_O_B  = input(1:3,1:3);
R_B_I  = input(1:3,4:6);
B_B_m  = input(1:3,7);
w_B_BI = input(1:3,8);

%% MAGNETIC VECOTR
B_B = B_B_m;

%% ANGULAR VELOCITY
w_O_OI = [0;-w_O;0];                % [rad/s] Angular Velocity of Orbit to Inertia Frame expressed in Orbit Frame
w_B_OI = R_O_B'*w_O_OI;             % [rad/s] Angular Velocity of Orbit to Inertia Frame expressed in Orbit Frame
w_B_BO = w_B_BI - w_B_OI;           % [rad/s] Angular Velocity of Body to Orbit Frame expressed in Body Frame

%% MAGNETIC DIPOLE
m_B     = -(K_d/norm(B_B,2)^2)*cross(B_B, w_B_BO);

[m_B,j] = currentScaling(m_B);

%% TORQUE
tau_m   = cross(m_B,B_B);

output(1:3,1) = tau_m;
end