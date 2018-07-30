function [f,torq,ss,q_d,w_d] = satellite_model(x,t,theta_d,phir_d,psir_d,d,j,jtrue,k,g,eps)
% Input
% x         - state variable [q1 q2 q3 q4 w1 w2 w3 h1 h2 h3]
% t         - instantaenous time 
% theta_d   - desired pitch angle
% phir_d    - desired roll rate
% psir_d    - desired yaw rate
% d         - disturbance
% j         - moment of inertia (assumed)
% jtrue     - moment of inertia (true)
% k         - sliding controller gain
% g         - G matrix
% eps       - boundary layer

% Pre-Allocate Space
f    = zeros(10,1);
q_d  = zeros(4,1);
w_d  = zeros(3,1);
w_dd = zeros(3,1);

% Quaternion, Angular Velocity and Wheel Speed
q  = [x(1);x(2);x(3);x(4)];
w  = [x(5);x(6);x(7)];
wh = [x(8);x(9);x(10)];

% Desired Roll and Yaw
phi_d = phir_d*t;
psi_d = psir_d*t;

% Desired Quaternion 
q_d(1) = sin(theta_d/2)*cos((phi_d-psi_d)/2);
q_d(2) = sin(theta_d/2)*sin((phi_d-psi_d)/2);
q_d(3) = cos(theta_d/2)*sin((phi_d+psi_d)/2);
q_d(4) = cos(theta_d/2)*cos((phi_d+psi_d)/2);

% Desired Angular Velocity and Derivative
w_d(1) = sin(theta_d)*sin(psi_d)*phir_d;
w_d(2) = sin(theta_d)*cos(psi_d)*phir_d;
w_d(3) = cos(theta_d)*phir_d+psir_d;

w_dd(1) = psir_d*sin(theta_d)*cos(psi_d)*phir_d;
w_dd(2) = -psir_d*sin(theta_d)*sin(psi_d)*phir_d;
w_dd(3) = 0;

% Cross Product Matrices 
wc = [  0   -w(3)  w(2);
       w(3)   0   -w(1);
      -w(2)  w(1)   0  ];
  
qc = [  0   -q(3)  q(2);
      q(3)    0   -q(1);
     -q(2)   q(1)   0  ];
 
qc_d = [  0    -q_d(3) q_d(2);
        q_d(3)    0   -q_d(1);
       -q_d(2)  q_d(1)   0   ];

% Xi Matrices
xiq   = [q(4)*eye(3)+qc;-q(1:3)'];
xiq_d = [q_d(4)*eye(3)+qc_d;-q_d(1:3)'];
dq    = [xiq_d'*q;q_d'*q];         

% Sliding Surface
ss = (w-w_d)+k*sign(dq(4,1))*dq(1:3,1);

% Sliding Mode Control
usat = [saturation(ss(1),eps);
        saturation(ss(2),eps);
        saturation(ss(3),eps)];
    
controller = 1;
switch controller
    case 1
    % [Crassidis] Optimal Variable Structure Control Tracking of Spacecraft Maneuvers
    dqd  = xiq_d'*xiq*w-xiq'*xiq_d*w_d;
    torq = wc*(j*w+wh)-j*(0.5*k*sign(dq(4,1))*dqd-w_dd + g*usat);
    
    case 2
    % [Crassidis] Optimal Variable Structure Control Tracking of Spacecraft Maneuvers (modified)   
    qd   = 0.5*xiq*w;               % qdot
    qd_d = 0.5*xiq_d*w_d;           % qdot_desired
    dqd  = xiq_d'*2*qd-xiq'*2*qd_d;
    torq = wc*(j*w+wh) - j*(0.5*k*sign(dq(4,1))*dqd - w_dd + g*usat);
    
    case 3
    % [Crassidis] Fundamentals Spacecraft Attitude Determination Control System
    torq = j*(0.5*k*(abs(dq(4,1))*(w_d-w)-sign(dq(4,1))*cross(dq(1:3,1),(w+w_d))) + w_dd - g*usat)+wc*(j*w+wh);
    
    case 4
    % [BST] to be continue
    dqd1 = qmul([w;0],dq);
    dqd2 = qmul(dq,[w_d;0]);
    dqd  = 0.5*(dqd1 - dqd2);
    torq = j*(w_dd -k*sign(dq(4,1))*dqd(1:3,1) - g*ss);

end
% Integration Functions
om   = [-wc w;
        -w' 0];
 
qdot = 0.5*om*q;                      % Satellite Kinetics
wdot = inv(jtrue)*(-wc*jtrue*w+torq); % Satellite Dynamics
hdot = -wc*wh-torq;                   % RW Angular Momentum

f(1:4,:)  = qdot; % Quaternion Rate
f(5:7,:)  = wdot; % Angular Acceleration
f(8:10,:) = hdot; % Wheel Angular Momentum Rate
