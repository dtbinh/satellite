close all
clear all
clc
global CONST

CONST.J = diag([0.056671585047683 0.057102938216026 0.0147]);

R_RW1_I = [ 0.707106781186548 ;-0.707106781186547;                 0]; % RW in Body Frame
R_RW2_I = [ 0.707106781186547 ; 0.707106781186548;                0];  % RW in Body Frame
R_RW3_I = [ 0                 ; 0                ;1.000000000000000];  % RW in Body Frame
R_RW4_I = [ -1                ; 0                ;                0];
D_I_RW = [R_RW1_I R_RW2_I R_RW3_I];

CONST.D_I_RW = D_I_RW;

D_RW_I = pinv(D_I_RW);

T_cmd = [0.1e-3;0;0];
I_rw  = 1.5465e-6;

%% TIME 
dt     = 0.01;          % [sec] time step
tdur   = 60;            % [sec] time at final
tspan  = 0:dt:tdur;     % [sec] time array
tlgth  = length(tspan); % [sec] time length

H_norm = [0;0;0];


q(:,1) = [0.7071;0;0;0.7071]; 
w(:,1) = [2*pi/60;0;0];
wh(:,1) = [0;0;0];
x(:,1) = [q(:,1);w(:,1);wh(:,1)];

for i=1:tlgth-1
    
Delta_H = wh(:,i) - H_norm;   

Tw_cmd = D_RW_I*T_cmd - (eye(3) - D_RW_I*D_I_RW)*Delta_H/dt;
    
% Runge Kutta Iterative Method
f1 = sat_model(x(:,i)           ,tspan(i)      ,Tw_cmd);
f2 = sat_model(x(:,i)+0.5*f1*dt,tspan(i)+0.5*dt,Tw_cmd);
f3 = sat_model(x(:,i)+0.5*f2*dt,tspan(i)+0.5*dt,Tw_cmd);
f4 = sat_model(x(:,i)+f3*dt    ,tspan(i)+dt    ,Tw_cmd);
x(:,i+1) = x(:,i)+1/6*(f1+2*f2+2*f3+f4)*dt;

% Actual Quaterion, Angular Velocity and Wheel Speed
q(:,i+1)     = x(1:4,i+1);
w(:,i+1)     = x(5:7,i+1);
wh(:,i+1)    = x(8:10,i+1);

R_B_I(:,:,i+1) = q2dcm(q(:,i+1),'xyzw');
track(i+1,:) = R_B_I(:,:,i+1)'*[0 ;0 ;1];

end

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
plotvector([1 ;0 ;0], [0 0 0], 'k', 'X',2);
plotvector([0 ;1 ;0], [0 0 0], 'k', 'Y',2);
plotvector([0 ;0 ;1], [0 0 0], 'k', 'Z',2);

% Satellite Body Frame
[X_sat,X_sat_lab] = plotvector(R_B_I(:,:,1)'*[1 ;0 ;0], [0 ;0 ;0], 'r','x_sat');
[Y_sat,Y_sat_lab] = plotvector(R_B_I(:,:,1)'*[0 ;1 ;0], [0 ;0 ;0], 'g','y_sat');
[Z_sat,Z_sat_lab] = plotvector(R_B_I(:,:,1)'*[0 ;0 ;1], [0 ;0 ;0], 'b','z_sat');

% Track
plot3(track(:,1),track(:,2),track(:,3))

for i=1:10:tlgth-1
updatevector(X_sat, R_B_I(:,:,i)'*[1 ;0 ;0], [0 ;0 ;0],1);
updatevector(Y_sat, R_B_I(:,:,i)'*[0 ;1 ;0], [0 ;0 ;0],1);
updatevector(Z_sat, R_B_I(:,:,i)'*[0 ;0 ;1], [0 ;0 ;0],1);   
    
set(X_sat_lab,'Position',R_B_I(:,:,i)'*[1 ;0 ;0]);
set(Y_sat_lab,'Position',R_B_I(:,:,i)'*[0 ;1 ;0]);
set(Z_sat_lab,'Position',R_B_I(:,:,i)'*[0 ;0 ;1]);
drawnow;     
end
    


