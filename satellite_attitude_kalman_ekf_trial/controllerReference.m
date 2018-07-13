function output=controllerReference(input)
%% GLOBAL PARAMETERS
global CONST
K_d = CONST.K_d;
K_p = CONST.K_p;
w_O = CONST.w_O;

%% INPUT
R_O_B  = input(1:3,1:3); % Rotation Matrix from Body to Orbit Frame
R_B_I  = input(1:3,4:6); % Rotation Matrix from Inertia to Body Frame
B_B_m  = input(1:3,7);   % Magnetic Field Vector (Measured) in Body Frame
w_B_BI = input(1:3,8);   % Angular Velocity of Body Frame with respect to the Inertia Frame
eps    = input(1:3,9);   % Quaternion xyz

%% MAGNETIC FIELD VECOTR
B_B    = B_B_m;          % Magnetic Field Vector in Body Frame

%% ANGULAR VELOCITY
w_O_OI = [0;-w_O;0];          % [rad/s] Angular Velocity of Orbit to Inertia Frame expressed in Orbit Frame
w_B_OI = R_O_B'*w_O_OI;       % [rad/s] Angular Velocity of Orbit to Inertia Frame expressed in Orbit Frame
w_B_BO = w_B_BI - w_B_OI;     % [rad/s] Angular Velocity of Body to Orbit Frame expressed in Body Frame

m_B    = -(K_d/norm(B_B,2)^2)*cross(B_B,w_B_BO)-(K_p/norm(B_B,2)^2)*cross(B_B,eps);

m_B    = currentScaling(m_B); % Scaling magnetic moment to prevent exceeding of Maximum Voltage
tau_m  = cross(m_B,B_B);      % [Nm] Torque provided

output(1:3,1) = tau_m;        % [Nm] Output
end