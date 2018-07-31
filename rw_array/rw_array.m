close all
clear all
clc
global CONST

CONST.J = diag([0.056671585047683 0.057102938216026 0.0147]);

R_B_RW1 = [ sqrt(2)/2  ;
                    0  ;                
            sqrt(2)/2 ]; % RW1 in Body Frame
R_B_RW2 = [         0  ; 
           -sqrt(2)/2  ;                
            sqrt(2)/2 ]; % RW2 in Body Frame
                
R_B_RW3 = [-sqrt(2)/2 ; 
                    0 ;
            sqrt(2)/2]; % RW3 in Body Frame
         
R_B_RW4 = [         0 ; 
            sqrt(2)/2 ;                
            sqrt(2)/2 ]; % RW4 in Body Frame

D_B_RW = [R_B_RW1 R_B_RW2 R_B_RW3 R_B_RW4];

CONST.D_B_RW = D_B_RW;

D_RW_B = pinv(D_B_RW);
CONST.D_RW_B = D_RW_B;

T_cmd = [0;0;0.1e-2];
I_rw  = 1.5465e-6;

%% TIME 
dt     = 0.01;          % [sec] time step
tdur   = 300;           % [sec] time at final
tspan  = 0:dt:tdur;     % [sec] time array
tlgth  = length(tspan); % [sec] time length

H_norm = [0;0;0;0];

q(:,1)  = [0;0;0;1]; 
w(:,1)  = [0;0;0];
wh(:,1) = [0;0;0];
w_h(:,1) = [0;0;0;0];
wdot(:,1) = [0;0;0];
x(:,1)  = [q(:,1);w(:,1);wh(:,1);w_h(:,1)];

for i=1:tlgth-1

% Control Torque Allocation 
Delta_H = w_h(:,i) - H_norm;   
Tw_cmd = D_RW_B*T_cmd - (eye(4) - D_RW_B*D_B_RW)*Delta_H/dt;
    
% Runge Kutta Iterative Method
f1 = sat_model(x(:,i)            ,tspan(i)         ,Tw_cmd);
f2 = sat_model(x(:,i) + 0.5*f1*dt,tspan(i) + 0.5*dt,Tw_cmd);
f3 = sat_model(x(:,i) + 0.5*f2*dt,tspan(i) + 0.5*dt,Tw_cmd);
f4 = sat_model(x(:,i) + f3*dt    ,tspan(i) + dt    ,Tw_cmd);
x(:,i+1) = x(:,i)+1/6*(f1+2*f2+2*f3+f4)*dt;

% Actual Quaterion, Angular Velocity and Wheel Speed
q(:,i+1)     = x(1:4,i+1);
w(:,i+1)     = x(5:7,i+1);
wdot(:,i+1) = (w(:,i+1)-w(:,i))/dt;
wh(:,i+1)    = x(8:10,i+1);
w_h(:,i+1)    = x(11:14,i+1);
R_B_I(:,:,i+1) = q2dcm(q(:,i+1),'xyzw');
track(i+1,:) = R_B_I(:,:,i+1)'*[0 ;0 ;1];

end
%% PLOT
close all
figure
subplot(3,1,1)
plot(tspan,w(1,:));
grid on; title('Angular Velocity of Satellite');
subplot(3,1,2)
plot(tspan,w(2,:));
grid on;
subplot(3,1,3)
plot(tspan,w(3,:));
grid on;

figure
subplot(3,1,1)
plot(tspan,CONST.J(1,1)*wdot(1,:));
grid on; title('Torque of Satellite');
subplot(3,1,2)
plot(tspan,CONST.J(2,2)*wdot(2,:));
grid on;
subplot(3,1,3)
plot(tspan,CONST.J(3,3)*wdot(3,:));
grid on;

figure
subplot(3,1,1)
plot(tspan,wh(1,:));
grid on;title('Angular Momentum of Satellite');
subplot(3,1,2)
plot(tspan,wh(2,:));
grid on;
subplot(3,1,3)
plot(tspan,wh(3,:));
grid on;

figure
subplot(4,1,1)
plot(tspan,w_h(1,:));
grid on;title('Angular Momentum of Wheel');
subplot(4,1,2)
plot(tspan,w_h(2,:));
grid on;
subplot(4,1,3)
plot(tspan,w_h(3,:));
grid on;
subplot(4,1,4)
plot(tspan,w_h(4,:));
grid on;
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

% Reaction Wheel
[rw1,rw1_lab] = plotvector(R_B_I(:,:,1)'*D_B_RW*[1 ;0 ;0 ;0], [0 ;0 ;0], 'k','rw_1');
[rw2,rw2_lab] = plotvector(R_B_I(:,:,1)'*D_B_RW*[0 ;1 ;0 ;0], [0 ;0 ;0], 'k','rw_2');
[rw3,rw3_lab] = plotvector(R_B_I(:,:,1)'*D_B_RW*[0 ;0 ;1 ;0], [0 ;0 ;0], 'k','rw_3');
[rw4,rw4_lab] = plotvector(R_B_I(:,:,1)'*D_B_RW*[0 ;0 ;0 ;1], [0 ;0 ;0], 'k','rw_4');



% Track
plot3(track(:,1),track(:,2),track(:,3))

for i=1:10:tlgth-1
updatevector(X_sat, R_B_I(:,:,i)'*[1 ;0 ;0], [0 ;0 ;0],1);
updatevector(Y_sat, R_B_I(:,:,i)'*[0 ;1 ;0], [0 ;0 ;0],1);
updatevector(Z_sat, R_B_I(:,:,i)'*[0 ;0 ;1], [0 ;0 ;0],1);   
    
set(X_sat_lab,'Position',R_B_I(:,:,i)'*[1 ;0 ;0]);
set(Y_sat_lab,'Position',R_B_I(:,:,i)'*[0 ;1 ;0]);
set(Z_sat_lab,'Position',R_B_I(:,:,i)'*[0 ;0 ;1]);

updatevector(rw1, R_B_I(:,:,i)'*D_B_RW*[1 ;0 ;0 ;0], [0 ;0 ;0],0.5);
set(rw1_lab,'Position',R_B_I(:,:,i)'*D_B_RW*[0.5 ;0 ;0 ;0]);

updatevector(rw2, R_B_I(:,:,i)'*D_B_RW*[0 ;1 ;0 ;0], [0 ;0 ;0],0.5);
set(rw2_lab,'Position',R_B_I(:,:,i)'*D_B_RW*[0 ;0.5 ;0 ;0]);

updatevector(rw3, R_B_I(:,:,i)'*D_B_RW*[0 ;0 ;1 ;0], [0 ;0 ;0],0.5);
set(rw3_lab,'Position',R_B_I(:,:,i)'*D_B_RW*[0 ;0 ;0.5 ;0]);

updatevector(rw4, R_B_I(:,:,i)'*D_B_RW*[0 ;0 ;0 ;1], [0 ;0 ;0],0.5);
set(rw4_lab,'Position',R_B_I(:,:,i)'*D_B_RW*[0 ;0 ;0 ;0.5]);
drawnow;     
end
    


