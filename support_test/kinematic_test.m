close all
clear all
clc

format long
%% 
X = [1;0;0];
Y = [0;1;0];
Z = [0;0;1];

% Figure Setting
fig = figure;
screensize = get(0,'ScreenSize');
set(fig,'Position',[0 0 screensize(4)*0.8 screensize(4)*0.8]);
grid on; axis fill;
cameratoolbar('SetMode','orbit')   
set(gca,'Position',[0 0 1 1]); % Set Position of Graph
set(gca,'CameraViewAngle',6); % Set Zoom of Graph
axis(3.5*[-1 1 -1 1 -1 1]);    % Set Limit of Axis  

% Earth Centered Inertial Frame
plotvector([1 ;0 ;0], [0 0 0], 'k', 'X_e_c_i');
plotvector([0 ;1 ;0], [0 0 0], 'k', 'Y_e_c_i');
plotvector([0 ;0 ;1], [0 0 0], 'k', 'Z_e_c_i');

% Transformation/Rotation from Frame I to Frame B
angle  = 45/180*pi;  % [rad] angle
vector = [1;0;1];    

q_B_I = [   vector(1)*sin(angle/2);
            vector(2)*sin(angle/2);
            vector(3)*sin(angle/2); 
                 cos(angle/2)];
             
q_B_I   = q_B_I./norm(q_B_I); % Normalisation of Quaternions 

eps = q_B_I(1:3);     % Euler Vector Component of Quaternion (xyzw)
eta = q_B_I(4);       % Euler Angle Component of Quaternion (xyzw)

S_eps = smtrx(eps);
     
R_B_I = eye(3) + 2*eta*S_eps + 2*S_eps^2 % [-] Transformation Matrix Body Frame to Orbit Frame 


%% q2dcm Function
R_B_I = q2dcm(q_B_I,'xyzw','tsf') % Quaternion Transformation from B to I


%% ATT Function

R_B_I = q2xi(q_B_I)'*q2psi(q_B_I)


%% QUATERNIONS PROPAGATION OF BODY WRT INERTIA FRAME
w_B_BI = [0.01;0.02;0.03];

eps = q_B_I(1:3);     % Euler Vector Component of Quaternion
eta = q_B_I(4);       % Euler Angle Component of Quaternion

S_eps = smtrx(eps);
eta_dot = -0.5* eps'*w_B_BI;               % Euler Angle Rate Component of Quaternion
eps_dot =  0.5*(eta*eye(3)+S_eps)*w_B_BI;  % Euler Vector Rate Component of Quaternion

q_dot_B_I = [eps_dot;eta_dot];           % Quaternion (xyzw) of Body Frame to Inertial Frame 

%% q2xi function

q_dot_B_I = 0.5*q2xi(q_B_I)*w_B_BI;
q2xi(q_B_I)
q2psi(q_B_I)
return
%% w2om function

q_dot_B_I = 0.5*w2om(w_B_BI)*q_B_I;

%% qmul & qmult Functions
angle1  = 90/180*pi;  % [rad] angle
vector1 = [0;0;1];    

q1_B_I = [   vector1(1)*sin(angle1/2);
             vector1(2)*sin(angle1/2);
             vector1(3)*sin(angle1/2); 
                 cos(angle1/2)];
angle2  = 80/180*pi;  % [rad] angle
vector2 = [0;1;0];    

q2_B_I = [   vector2(1)*sin(angle2/2);
             vector2(2)*sin(angle2/2);
             vector2(3)*sin(angle2/2); 
                  cos(angle2/2)];
% rotation function
R_B_I =dcm(2,angle2)* dcm(3,angle1)
plotvector(R_B_I(:,:,1)'*[1 ;0 ;0], [0 0 0], 'm', 'x_d_c_m');
plotvector(R_B_I(:,:,1)'*[0 ;1 ;0], [0 0 0], 'm', 'y_d_c_m');
plotvector(R_B_I(:,:,1)'*[0 ;0 ;1], [0 0 0], 'm', 'z_d_c_m');

% qmul function
qout_B_I = qmul(q1_B_I,q2_B_I,'*')
R_B_I = q2dcm(qout_B_I)

plotvector(R_B_I(:,:,1)'*[1 ;0 ;0], [0 0 0], 'b', 'x_q_m_u_l*');
plotvector(R_B_I(:,:,1)'*[0 ;1 ;0], [0 0 0], 'b', 'y_q_m_u_l*');
plotvector(R_B_I(:,:,1)'*[0 ;0 ;1], [0 0 0], 'b', 'z_q_m_u_l*');

% qmul function
qout_B_I = qmul(q1_B_I,q2_B_I,'.')
R_B_I = q2dcm(qout_B_I)

plotvector(R_B_I(:,:,1)'*[1 ;0 ;0], [0 0 0], 'b', 'x_q_m_u_l.');
plotvector(R_B_I(:,:,1)'*[0 ;1 ;0], [0 0 0], 'b', 'y_q_m_u_l.');
plotvector(R_B_I(:,:,1)'*[0 ;0 ;1], [0 0 0], 'b', 'z_q_m_u_l.');


