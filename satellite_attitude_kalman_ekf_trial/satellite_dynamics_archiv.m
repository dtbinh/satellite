function output = satellite_dynamics(input)
global CONST;

w_O = CONST.w_O;   % [rad/s] Orbit Angular Velocity
I   = CONST.I;     % [kgm^2] Spacecraft Moments of Inertia

%% INPUTS
T      = input(1:3);    % [Nm] External Torque
q_B_O  = input(4:7);    % [-] Quaternions(wxyz) Body Frame to Orbit Frame
w_B_BI = input(8:10);   % [rad/s] Angular Velocity of Body ref to Inertia in Body frame
q_B_I  = input(11:14);  % [-] Quaternions(wxyz) Body Frame to Inertial Frame

%% MATRIX TRANSFORMATION FROM QUATERNIONS
q_B_O   = q_B_O./norm(q_B_O); % Normalisation of Quaternions 
q_B_I   = q_B_I./norm(q_B_I); % Normalisation of Quaternions 
 
eta = q_B_O(1);       % Euler Angle Component of Quaternion q = [eta eps1 eps2 eps3]
eps = q_B_O(2:4);     % Euler Vector Component of Quaternion q = [eta eps1 eps2 eps3]

S_eps = [     0    -eps(3)   eps(2);
          eps(3)       0    -eps(1);
         -eps(2)   eps(1)        0 ];
     
R_O_B = eye(3) + 2*eta*S_eps + 2*S_eps^2; % [-] Transformation Matrix Body Frame to Orbit Frame 
R_B_O = R_O_B';                           % [-] Transformation Matrix Orbit Frame to Body Frame

w_O_OI = [0;-w_O;0];                      % [rad/s] Angular Velocity of Orbit to Inertia Frame expressed in Orbit Frame
w_B_OI = R_B_O*w_O_OI ;                   % [rad/s] Angular Velocity of Orbit to Inertia Frame expressed in Orbit Frame
w_B_BO = w_B_BI - w_B_OI;                 % [rad/s] Angular Velocity of Body to Orbit Frame expressed in Body Frame

%% SATELLITE DYNAMICS
wdot_B_BI = I^(-1)*(T-cross(w_B_BI,I*w_B_BI)); % [rad/s] Angular Velocity of Body to Inertia expressed in Body Frame

%% QUATERNIONS PROPAGATION OF BODY FRAME WRT ORBIT FRAME
eta_dot = -0.5* eps'*w_B_BO;              % Euler Angle Rate Component of Quaternion
eps_dot =  0.5*(eta*eye(3)+S_eps)*w_B_BO; % Euler Vector Rate Component of Quaternion

q_dot_B_O = [eta_dot;eps_dot];            % Rotation Quaternion of Body Frame to Orbital Frame

%% QUATERNIONS PROPAGATION OF BODY WRT INERTIA FRAME
eta = q_B_I(1);       % Euler Angle Component of Quaternion
eps = q_B_I(2:4);     % Euler Vector Component of Quaternion

S_eps = [     0    -eps(3)   eps(2);
          eps(3)       0    -eps(1);
         -eps(2)   eps(1)        0 ];

eta_dot = -0.5* eps'*w_B_BI;               % Euler Angle Rate Component of Quaternion
eps_dot =  0.5*(eta*eye(3)+S_eps)*w_B_BI;  % Euler Vector Rate Component of Quaternion

q_dot_B_I = [eta_dot;eps_dot];             % Rotation Quaternion of Body Frame to Inertial Frame

%% OUTPUT
output = [q_dot_B_O ; 
          wdot_B_BI ;
          q_dot_B_I ];