% This program tracks an attitude trajectory using sliding mode control.
% The WMAP spacecraft is the model used for the simulation.
%
% Fundamentals of Spacecraft Attitude Determination and Control by Markley and Crassidis
% Example 7.3

% Other Required Routines: satellite_model.m, saturation.m

% Written by John L. Crassidis 8/13

close all
clear all
clc

% True Inertia 
intrue=[ 399 ,-2.81,-1.31;
        -2.81,  377, 2.54;
        -1.31, 2.54, 377 ];

% Assumed Inertia
in = [  380,-2.90,  -1.30;
       -2.90,  360,  2.50;
       -1.30, 2.50,  340 ];

% Time 
dt = 0.1;        % [sec] Sample Time
tf = 3600;       % [sec] Final Time
t  = [0:dt:tf]'; % [sec] Time Array
m  = length(t);  % [sec] Time Array length

% Pre-Allocate Space
q     = zeros(m,4);
q_d   = zeros(m,4);
w_d   = zeros(m,3);
w_dd  = zeros(m,3);
w     = zeros(m,3);
u     = zeros(m,3);
wheel = zeros(m,3);
slide = zeros(m,3);
xa    = zeros(m,10);
i1000 = 0;

% Disturbance Torque
dist = [0.005*sin(0.05*t) 0.003*ones(m,1) 0.005*cos(0.05*t)]; 

null = 1; % Set null=1 for Tracking and null=0 for Regulation 

% To provide the scan pattern, 
%  - spacecraft spins about the z-axis at 0.464 rpm (Yaw Rate)
%  - the z-axis cones about the Sunline at 1 rev/h. (Roll Rate)
%  - A 22.5deg+-0.25deg angle between the z-axis and the Sun direction for
%    constant power input and constant temperatures for alignment stability.

% Desired Euler Quantities
phir_d  = 1*(2*pi/3600)*null; % [rad/s] Roll Rate
theta_d = 22.5*pi/180*null;   % [rad]   Pitch Angle
psir_d  = 0.464*2*pi/60*null; % [rad/s] Yaw Rate

fprintf('Desired Roll Rate   : %.8f [rad/s]\n',phir_d )
fprintf('Desired Yaw Rate    : %.8f [rad/s]\n',psir_d )
fprintf('Desired Pitch Angle : %.8f [rad]\n',theta_d )

% Get Euler Angles for Each Time Step(zero conditions)
phi_d = phir_d.*t*null;
psi_d = psir_d.*t*null;

% Initial Desired Quaternion
q_d(1,1) = sin(theta_d/2).*cos((phi_d(1)-psi_d(1))/2);
q_d(1,2) = sin(theta_d/2).*sin((phi_d(1)-psi_d(1))/2);
q_d(1,3) = cos(theta_d/2).*sin((phi_d(1)+psi_d(1))/2);
q_d(1,4) = cos(theta_d/2).*cos((phi_d(1)+psi_d(1))/2);

% Initial Desired Angular Velocity
w_d(1,1) = sin(theta_d).*sin(psi_d(1)).*phir_d;
w_d(1,2) = sin(theta_d).*cos(psi_d(1)).*phir_d;
w_d(1,3) = cos(theta_d).*phir_d+psir_d;

% Initial Quaternion, Angular Velocity and Wheel Speed
qc_d =[   0    -q_d(1,3)  q_d(1,2);
       q_d(1,3)   0      -q_d(1,1);
      -q_d(1,2) q_d(1,1)     0   ];

xiq_d  = [q_d(1,4)*eye(3)+qc_d;-q_d(1,1:3)];
q(1,:) = ([xiq_d q_d(1,:)']*[0;0;sin(60/2*pi/180);cos(60/2*pi/180)])';
q(1,:) = [0.7071;0;0;0.7071];
w(1,:) = [0.001;0.003;0.0003];
w(1,:) = [0;0;0];
xa(1,:)= [q(1,:) w(1,:) 0 0 0];

fprintf('Initial Quaternion  : %.4f %.4f %.4f %.4f  [ ]\n',q(1,:))
fprintf('Initial Ang Velocity: %.4f %.4f %.4f          [rad/s]     \n',w(1,:))

% Gains
k   = 0.015;
g   = 0.15*eye(3);
eps = 0.01;
  
% Initial Torque, Sliding Manifold, Desired Quaternion and Angular Velocity
[ff,torq,ss,qq,ww] = satellite_model(xa(1,:),t(1),theta_d,phir_d,psir_d,dist(1,:),in,intrue,k,g,eps);
u(1,:)     = torq(:)';
slide(1,:) = ss(:)';
q_d(1,:)   = qq(:)';
w_d(1,:)   = ww(:)';
R_I_B(:,:,1) = q2dcm(q(1,:)','xyzw');
track(1,:)   = R_I_B(:,:,1)'*[0 ;0 ;1];

% Main Loop
for i=1:m-1

% Runge Kutta Iterative Method
f1 = satellite_model(xa(i,:)           ,t(i)       ,theta_d,phir_d,psir_d,dist(i,:),in,intrue,k,g,eps);
f2 = satellite_model(xa(i,:)+0.5*f1'*dt,t(i)+0.5*dt,theta_d,phir_d,psir_d,dist(i,:),in,intrue,k,g,eps);
f3 = satellite_model(xa(i,:)+0.5*f2'*dt,t(i)+0.5*dt,theta_d,phir_d,psir_d,dist(i,:),in,intrue,k,g,eps);
f4 = satellite_model(xa(i,:)+f3'*dt    ,t(i)+dt    ,theta_d,phir_d,psir_d,dist(i,:),in,intrue,k,g,eps);
xa(i+1,:) = xa(i,:)+1/6*(f1'+2*f2'+2*f3'+f4')*dt;

% Get Torque, Sliding Manifold, Desired Quaternion and Angular Velocity
[ff,torq,ss,qq,ww] = satellite_model(xa(i+1,:),t(i+1,:),theta_d,phir_d,psir_d,dist(i+1,:),in,intrue,k,g,eps);
u(i+1,:) = torq(:)';
slide(i+1,:) = ss(:)';
q_d(i+1,:) = qq(:)';
w_d(i+1,:) = ww(:)'; 

% Actual Quaterion, Angular Velocity and Wheel Speed
q(i+1,:) = xa(i+1,1:4);
w(i+1,:) = xa(i+1,5:7);
wheel(i+1,:) = xa(i+1,8:10);

R_I_B(:,:,i+1) = q2dcm(q(i+1,:)','xyzw');
track(i+1,:) = R_I_B(:,:,i+1)'*[0 ;0 ;1];

end

% Get Roll, Pitch and Yaw (3-1-3 sequence)
phi   = atan2(2*(q(:,1).*q(:,3)+q(:,2).*q(:,4)), -2*(q(:,2).*q(:,3)-q(:,1).*q(:,4)));
theta = acos(-q(:,1).^2-q(:,2).^2 + q(:,3).^2+q(:,4).^2);
psi   = atan2(2*(q(:,1).*q(:,3)-q(:,2).*q(:,4)),2*(q(:,2).*q(:,3)+q(:,1).*q(:,4)));
werr  = (w_d-w)*180/pi;
erre  = [phi_d-unwrap(phi) theta_d-theta psi_d-(unwrap(psi)+2*pi)]*180/pi;
erre(:,3) = erre(:,3)+360;

% Plots
clf
subplot(311)
plot(t/60,erre(:,1));
set(gca,'fontsize',12);
axis([0 60 -2 2])
set(gca,'ytick',[-2 -1 0 1 2])
ylabel('Roll (Deg)');

subplot(312)
plot(t/60,erre(:,2));
set(gca,'fontsize',12);
axis([0 60 -4 4])
set(gca,'ytick',[-4 -2 0 2 4])
ylabel('Pitch (Deg)');

subplot(313)
plot(t/60,erre(:,3));
set(gca,'fontsize',12);
axis([0 60 -60 30])
set(gca,'ytick',[-60 -30 0 30])
ylabel('Yaw (Deg)');
xlabel('Time (Min)');

figure

subplot(311)
plot(t/60,werr(:,1));
set(gca,'fontsize',12);
axis([0 60 -0.02 0.02])
set(gca,'ytick',[-0.02 -0.01 0 0.01 0.02])
ylabel('dw1 (Deg/Sec)');

subplot(312)
plot(t/60,werr(:,2));
set(gca,'fontsize',12);
axis([0 60 -0.02 0.02])
set(gca,'ytick',[-0.02 -0.01 0 0.01 0.02])
ylabel('dw2 (Deg/Sec)');

subplot(313)
plot(t/60,werr(:,3));
set(gca,'fontsize',12);
axis([0 60 -0.2 0.4])
set(gca,'ytick',[-0.2 0 0.2 0.4])
ylabel('dw3 (Deg/Sec)');
xlabel('Time (Min)');

figure

subplot(311)
plot(t/60,wheel(:,1));
set(gca,'fontsize',12);
axis([0 20 -0.4 0.4])
set(gca,'ytick',[-0.4 -0.2 0 0.2 0.4])
ylabel('h1 (Nms)');

subplot(312)
plot(t/60,wheel(:,2));
set(gca,'fontsize',12);
axis([0 20 -0.4 0.2])
set(gca,'ytick',[-0.4 -0.2 0 0.2])
ylabel('h2 (Nms)');

subplot(313)
plot(t/60,wheel(:,3));
set(gca,'fontsize',12);
axis([0 20 -20 -16])
set(gca,'ytick',[-20 -19 -18 -17 -16])
ylabel('h3 (Nms)');
xlabel('Time (Min)');

figure

d_norm = (dist(:,1).^2+dist(:,2).^2+dist(:,3).^2).^(0.5);
d_norm_max = max(d_norm);
bound = norm(eps*inv(in*g),'fro')*d_norm_max*ones(m,1);
slide_norm = (slide(:,1).^2+slide(:,2).^2+slide(:,3).^2).^(0.5);
clf
plot(t/60,slide_norm,t/60,bound)
set(gca,'fontsize',12);
axis([0 60 0 3e-6])
ylabel('Slide Norm and Bound');
xlabel('Time (Min)');

figure

% Scan Plot
e1 = cos(theta_d).*cos(psi_d).*cos(phi_d)-cos(theta_d).^2.*sin(psi_d).*sin(phi_d)+sin(theta_d).^2.*sin(phi_d);
e2 = cos(theta_d).*cos(psi_d).*sin(phi_d)+cos(theta_d).^2.*sin(psi_d).*cos(phi_d)-sin(theta_d).^2.*cos(phi_d);
e3 = cos(theta_d).*sin(theta_d).*(sin(psi_d)+1);
x = e1./(1+e3);y=e2./(1+e3);
plot(x,y);
set(gca,'fontsize',12);
set(gca,'XTicklabels',[])
set(gca,'YTicklabels',[])
set(gca,'XTick',[])
set(gca,'YTick',[])
set(gca,'DataAspectRatio',[1 1 1])
axis([-1.05 1.05 -1.05 1.05])
%% SIMULATION
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
asun = plotvector([0 ;0 ;1], [0 0 0], 'k', 'Anti-Sun Vector',2);

% Satellite Body Frame
[X_sat,X_sat_lab] = plotvector(R_I_B(:,:,1)'*[1 ;0 ;0], [0 ;0 ;0], 'r');
[Y_sat,Y_sat_lab] = plotvector(R_I_B(:,:,1)'*[0 ;1 ;0], [0 ;0 ;0], 'g');
[Z_sat,Z_sat_lab] = plotvector(R_I_B(:,:,1)'*[0 ;0 ;1], [0 ;0 ;0], 'b');

% Track
plot3(track(:,1),track(:,2),track(:,3))

for i=1:10:m-1
updatevector(X_sat, R_I_B(:,:,i)'*[1 ;0 ;0], [0 ;0 ;0],1);
updatevector(Y_sat, R_I_B(:,:,i)'*[0 ;1 ;0], [0 ;0 ;0],1);
updatevector(Z_sat, R_I_B(:,:,i)'*[0 ;0 ;1], [0 ;0 ;0],1);   
    

drawnow;     
end
    
