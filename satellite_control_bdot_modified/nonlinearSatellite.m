function xDot = nonlinearSatellite(t,x,P)
%% INPUT VARIABLES FROM SOLVER
w_B_IB = x(1:3);  % [rad/s] Differentiate of wdot_B_IB           
q_B_O  = x(4:7);  % []      Differentiate of eta_dot and eps_dot [n e1 e2 e3]
lat    = x(8);    % [rad]   Differentiate of w_O

%% INPUT VARIABLE FROM PARAMETERS
global CONST

w_O    = CONST.w_O;   % [s] Orbit Velocity
I      = CONST.I;     % [kgm^2] Moment of Inertia

q_B_O   = q_B_O./norm(q_B_O); % Normalisation of quaternions

eta = q_B_O(1);       % Euler Angle Component of Quaternion
eps = q_B_O(2:4);     % Euler Vector Component of Quaternion
%% MATRIX TRANSFORMATION FROM QUATERNIONS
S_eps = [     0    -eps(3)   eps(2);
          eps(3)       0    -eps(1);
         -eps(2)   eps(1)        0 ];
     
R_O_B = eye(3) + 2*eta*S_eps + 2*S_eps^2; % Rotation Matrix Body Frame to Orbit Frame
R_B_O = R_O_B';                           % Rotation Matrix Orbit Frame to Body Frame

w_O_OI = [0;-w_O;0];                % [rad/s] Angular Velocity of Orbit to Inertia Frame expressed in Orbit Frame
w_B_OI = R_B_O*w_O_OI;              % [rad/s] Angular Velocity of Orbit to Inertia Frame expressed in Orbit Frame
w_B_BO = w_B_IB - w_B_OI;           % [rad/s] Angular Velocity of Body to Orbit Frame expressed in Body Frame
%% MAGNETIC FIELD
[B_O,Bdot_O] = magneticFieldDipole(P,lat);
% [B_O]  = magneticFieldIGRF(10,10,lat,0,P);
B_B    = R_B_O*B_O;

%% CONTROLLER
 [tau_m,j] = controllerDissipative(P,w_B_BO,B_B);
% [tau_m,j] = controllerReference(P,w_B_BO,B_B,q);
% [tau_m,j] = controllerBdot(P,w_B_BO,B_O,R_B_O,Bdot_O);


%% GRAVITY TORQUE
c3    = R_B_O(:,3);             % Obtain the Z-axis of Orbit Frame into Body Frame
tau_g = 3*w_O^2*cross(c3,I*c3); 
%% SATELLITE DYNAMICS
wdot_B_IB = I^(-1)*(tau_m+tau_g-cross(w_B_IB,I*w_B_IB));

%% QUATERNIONS PROPAGATION
eta_dot = -0.5*eps'*w_B_BO;
eps_dot =  0.5*(eta*eye(3)+S_eps)*w_B_BO;

%% OUTPUT
xDot(1:3,1) = wdot_B_IB;
xDot(4,1)   = eta_dot;
xDot(5:7,1) = eps_dot;
xDot(8,1)   = w_O;
xDot(9,1)   = j;
%% STORING VARIABLE
global tmpVAR
tmpVAR.torque = tau_m;

end