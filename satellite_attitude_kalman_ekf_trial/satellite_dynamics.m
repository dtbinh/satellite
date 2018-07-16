function output = satellite_dynamics(input)
global CONST;

w_O = CONST.w_O;   % [rad/s] Orbit Angular Velocity
I   = CONST.I;     % [kgm^2] Spacecraft Moments of Inertia

%% INPUTS
T      = input(1:3);    % [Nm] External Torque
q_B_O  = input(4:7);    % [-] Quaternions(xyzw) Body Frame to Orbit Frame
w_B_BI = input(8:10);   % [rad/s] Angular Velocity of Body ref to Inertia in Body frame
q_B_I  = input(11:14);  % [-] Quaternions(xyzw) Body Frame to Inertial Frame

%% MATRIX TRANSFORMATION FROM QUATERNIONS
q_B_O   = qnorm(q_B_O); % Normalisation of Quaternions 
q_B_I   = qnorm(q_B_I); % Normalisation of Quaternions 
    
R_B_O = q2xi(q_B_O)'*q2psi(q_B_O); % [-] Transformation Matrix Orbit Frame to Body Frame

w_O_OI = [0;-w_O;0];               % [rad/s] Angular Velocity of Orbit to Inertia Frame expressed in Orbit Frame
w_B_OI = R_B_O*w_O_OI ;            % [rad/s] Angular Velocity of Orbit to Inertia Frame expressed in Orbit Frame
w_B_BO = w_B_BI - w_B_OI;          % [rad/s] Angular Velocity of Body to Orbit Frame expressed in Body Frame

%% SATELLITE DYNAMICS
wdot_B_BI = I^(-1)*(T-cross(w_B_BI,I*w_B_BI)); % [rad/s] Angular Velocity of Body to Inertia expressed in Body Frame

%% QUATERNIONS PROPAGATION OF BODY FRAME WRT ORBIT FRAME
q_dot_B_O = 0.5*q2xi(q_B_O)*w_B_BO;            % Quaternion (xyzw)of Body Frame to Orbital Frame

%% QUATERNIONS PROPAGATION OF BODY WRT INERTIA FRAME
q_dot_B_I = 0.5*q2xi(q_B_I)*w_B_BI;             % Quaternion (xyzw) of Body Frame to Inertial Frame 

%% OUTPUT
output = [q_dot_B_O ;  % [xyzw] Quaternion Rate of Body Frame to Orbital Frame
          wdot_B_BI ;  % [rad/s] Angular Velocity of Body to Inertia expressed in Body Frame
          q_dot_B_I ]; % [xyzw] Quaternion Rate of Body Frame to Inertial Frame