function output = satellite_dynamics(input)
global CONST;

I   = CONST.I;     % [kgm^2] Spacecraft Moments of Inertia

%% INPUTS
T      = input(1:3);    % [Nm] External Torque
q_B_I  = input(4:7);    % [-] Quaternions(xyzw) Body Frame to Inertial Frame
w_B_BI = input(8:10);   % [rad/s] Angular Velocity of Body ref to Inertia in Body frame

%% MATRIX TRANSFORMATION FROM QUATERNIONS
q_B_I   = qnorm(q_B_I); % Normalisation of Quaternions 

%% SATELLITE DYNAMICS
wdot_B_BI = I^(-1)*(T-cross(w_B_BI,I*w_B_BI)); % [rad/s] Angular Velocity of Body to Inertia expressed in Body Frame

%% SATELLITE KINEMATICS
q_dot_B_I = 0.5*q2xi(q_B_I)*w_B_BI;             % Quaternion (xyzw) of Body Frame to Inertial Frame 

%% OUTPUT
output = [q_dot_B_I ;  % [xyzw] Quaternion Rate of Body Frame to Inertial Frame
          wdot_B_BI ];  % [rad/s] Angular Velocity of Body to Inertia expressed in Body Frame
           