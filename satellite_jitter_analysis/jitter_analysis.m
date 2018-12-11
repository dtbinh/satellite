% Jitter Analysis of Spacecraft

close all
clear all
clc

% ------------------------------------------------------------------------
% Parameters
N_RW  = 3;    % [ ] Number of Reaction Wheels
m_sc  = 680;  % [kg] Mass of Spacecraft
m_hub = 644;  % [kg] Wheel Mass   
m_rw  = 12;

I_hub_Bc = [  550  0.1045 -0.0840; 
             0.1045  650   0.0001;
            -0.0840 0.0001  650];
        
r_Bc_B   = [1 ; -2; 10]; % [cm] Hub Center of Mass wrt to Body
G_s      = [ 0.7887 -0.2113 -0.5774;
            -0.2113  0.7887 -0.5774;
             0.5774  0.5774  0.5774 ];

U_s  = 0.48;  % [g.cm] Wheel Static Imbalance
U_d  = 15.4;  % [g.cm2] Wheel dynamic Imbalance

d    = 0.4e-6;   % [m] Wheel Center of Mass Offset (derived from Us)
I_rw_Wc = [1.5915   0    1.54e-6;
              0   0.8594    0   ;
           1.54e-6  0    0.8594 ]; % [kg.m2]

r_w1_B = [0.6309 ; -0.1691; 0.4619]; % [m] 
r_w2_B = [-0.1691;  0.6309; 0.4619]; % [m]
r_w3_B = [-0.4619; -0.4619; 0.4619]; % [m]

r_B_N = [0;0;0];   % [m]  initial position of spacecraft
v_B_N = [0;0;0];   % [m]  initial velocity of spacecraft
sig_B_N = [0;0;0]; % [ ]  initial attitude MRP of spacecraft
w_B_N = [0;0;0];   % [dps] initial angular velocity of spacecraft

om_    = [-558; -73; 242]; % [rpm] initial wheel speeds
theta = [ 43 ; 179; 346]; % [deg] initial wheel angles
us    = [200 ;-500;350];  % [nNm] commanded wheel torques

% -----------------------------------------------------------------------
% Simulation Time 
dt = 0.1;        % [sec] Sample Time
tf = 2400;       % [sec] Final Time
t  = [0:dt:tf]'; % [sec] Time Array
N_t= length(t);  % [sec] Time Array length

% -----------------------------------------------------------------------
% Satellite Parameters
sat.I    = [ 399 , 0,   0;
              0,  377,  0;
              0,   0,  377 ];
sat.I_rw = 1;
% -----------------------------------------------------------------------
% Initial Parameters
q(1,:)  = [0.0;0;0;1.0]; % Initial Quaternion
w(1,:)  = [0.0;0.0;0.0];       % [rad/s] Initial Angular Velocity
om(1,:) = [-4.0;2.0;1.0];       % [rad/s] Initial Wheel Speed
hw(1,:) = sat.I_rw*om(1,:);    % [rad/s] Initial Reaction Wheel Momentum 
x(1,:)  = [q(1,:) w(1,:) hw(1,:)];

R_I_B(:,:,1) = q2dcm(q(1,:)','xyzw');
track(1,:)   = R_I_B(:,:,1)'*[0 ;0 ;1];

% ------------------------------------------------------------------------
for i=1:1:N_t-1
% Control Target
ctrl.trq = [0.0; 0.0; 0.0]; % desired torque

% Runge Kutta Iterative Method
f1 = satellite_model(x(i,:)           ,t(i)       ,sat,ctrl);
f2 = satellite_model(x(i,:)+0.5*f1'*dt,t(i)+0.5*dt,sat,ctrl);
f3 = satellite_model(x(i,:)+0.5*f2'*dt,t(i)+0.5*dt,sat,ctrl);
f4 = satellite_model(x(i,:)+f3'*dt    ,t(i)+dt    ,sat,ctrl);
x(i+1,:) = x(i,:)+1/6*(f1'+2*f2'+2*f3'+f4')*dt;

% Get Torque, Sliding Manifold, Desired Quaternion and Angular Velocity
ff = satellite_model(x(i+1,:),t(i+1,:),sat,ctrl);

% Actual Quaterion, Angular Velocity and Wheel Speed
q(i+1,:) = x(i+1,1:4);
w(i+1,:) = x(i+1,5:7);
hw(i+1,:) = x(i+1,8:10);

R_I_B(:,:,i+1) = q2dcm(q(i+1,:)','xyzw');
track(i+1,:) = R_I_B(:,:,i+1)'*[0 ;0 ;1];
end

% ------------------------------------------------------------------------
% SIMULATION

% Figure Setting
fig = figure;
screensize = get(0,'ScreenSize');
set(fig,'Position',[0 0 screensize(4)*0.8 screensize(4)*0.8]);
grid on; axis fill;
cameratoolbar('SetMode','orbit')   
set(gca,'Position',[0 0 1 1]); % Set Position of Graph
set(gca,'CameraViewAngle',6);  % Set Zoom of Graph
axis(3.5*[-1 1 -1 1 -1 1]);    % Set Limit of Axis 

% Earth Centered Inertial Frame
plotvector([1 ;0 ;0], [0 0 0], color('grey'), 'X',2);
plotvector([0 ;1 ;0], [0 0 0], color('grey'), 'Y',2);
plotvector([0 ;0 ;1], [0 0 0], color('grey'), 'Z',2);

% Satellite Body Frame
[X_sat,X_sat_lab] = plotvector(R_I_B(:,:,1)'*[1 ;0 ;0], [0 ;0 ;0], 'r');
[Y_sat,Y_sat_lab] = plotvector(R_I_B(:,:,1)'*[0 ;1 ;0], [0 ;0 ;0], 'g');
[Z_sat,Z_sat_lab] = plotvector(R_I_B(:,:,1)'*[0 ;0 ;1], [0 ;0 ;0], 'b');

% Track
plot3(track(:,1),track(:,2),track(:,3))

for i=1:50:N_t-1
updatevector(X_sat, R_I_B(:,:,i)'*[1 ;0 ;0], [0 ;0 ;0],1);
updatevector(Y_sat, R_I_B(:,:,i)'*[0 ;1 ;0], [0 ;0 ;0],1);
updatevector(Z_sat, R_I_B(:,:,i)'*[0 ;0 ;1], [0 ;0 ;0],1);   
    

drawnow;     
end
    
