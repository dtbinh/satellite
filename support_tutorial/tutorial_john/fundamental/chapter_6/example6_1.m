% This example shows a simulation using a typical star camera 
% with gyro measurements to determine the attitude of a rotating 
% spacecraft using an extended Kalman filter. This program assumes
% quaternion measurements and also runs a smoother to determine 
% the gyro calibration parameters.

close all
clear all
clc

% Time 
dt = 1;
tf = 90*60;
t  = [0:dt:tf]';
m  = length(t);

% True Rate Quaternion
w      = 0.1*pi/180*[sin(0.01*t) sin(0.0085*t) cos(0.0085*t)];
q      = zeros(m,4);          % Prepare
q(1,:) = 1/sqrt(2)*[1 0 0 1]; % Initialise

% Obtaining the true quaternion
for i=1:m-1
 psik  =  sin(1/2*norm(w(i,:)')*dt)/norm(w(i,:)')*w(i,:)';
 omega = [cos(1/2*norm(w(i,:)')*dt)*eye(3)-smtrx(psik)       psik;
                    -psik'                       cos(1/2*norm(w(i,:)')*dt) ];
   
 q(i+1,:)=(omega*q(i,:)')';
end

%%
% Quaternion Measurements Directly 
sig_tracker = 6/3600*pi/180;         % Noise sigma for Star Tracker 
st_noise = 0.5*sig_tracker*randn(m,3); % Generate noise

qm = quat_mult(q,[st_noise ones(m,1)]); % This program multiplies two quaternion sets.

% Normalise Quaternion
qmnorm = (qm(:,1).^2+qm(:,2).^2+qm(:,3).^2+qm(:,4).^2).^(0.5);
qm(:,1) = qm(:,1)./qmnorm;
qm(:,2) = qm(:,2)./qmnorm;
qm(:,3) = qm(:,3)./qmnorm;
qm(:,4) = qm(:,4)./qmnorm;


% True Scale Factors and Misalignments
s = 1e-6*[1500 1000 1500;
           500 1000 2000;
           1000 1500 1500];

%% Get True Rates and Measure Rates
wtrue = w;
sigu  = sqrt(10)*1e-10;
sigv  = sqrt(10)*1e-7;
num_g = dt*[1 1];
den_g = 2*[1 -1];
[phi_g,gam_g,c_g,d_g] = tf2ss(num_g,den_g);

% Bias
bias0 = [10 10 10]/180*pi/3600;
bias1 = dlsim(phi_g,gam_g,c_g,d_g,sigu/sqrt(dt)*randn(m,1),bias0(1)/dt);
bias2 = dlsim(phi_g,gam_g,c_g,d_g,sigu/sqrt(dt)*randn(m,1),bias0(2)/dt);
bias3 = dlsim(phi_g,gam_g,c_g,d_g,sigu/sqrt(dt)*randn(m,1),bias0(3)/dt);
bias = [bias1 bias2 bias3];

wgm = ((eye(3)+s)*wtrue')'+sqrt(sigv^2/dt+1/12*sigu^2*dt)*randn(m,3)+bias;

%% Initial Conditions and Covariances
x0   = [q(1,:)';zeros(12,1)];      % Initial Values for states
poa  = (0.1*pi/180)^2*eye(3);      % Initial Covariance Value Attitude
pog  = (0.2*pi/180/3600)^2*eye(3); % Initial Covariance Value Gyro Bias
pos  = (2000*1e-6/3)^2*eye(3);     % Initial Covariance Value
poku = (2000*1e-6/3)^2*eye(3);     % Initial Covariance Value
pokl = (2000*1e-6/3)^2*eye(3);     % Initial Covariance Value

%% Extended Kalman Filter 
[qe,xe,pcov,pp_store,pm_store,phi_store,xp_store,xm_store]=kal_gyro_quatd_15(qm,wgm,sig_tracker^2*eye(3),sigu,sigv,poa,pog,pos,poku,pokl,dt,x0);

%% Smoother
ps    = shiftdim(pm_store(m,:,:),1); % Obtain the Error Covariance at the last step

pscov(m,:)= diag(ps)';               % Save the diagonals of the error covariance matrix

xe_s(m,:) = shiftdim(xm_store(m,:,:),1); % Last row value of state estimate
qe_s(m,:) = qe(m,:);                     % Last row value of quaternion estimate

% Compute Attitude Errors and 3-Sigma Outlier
qerr = quat_err(qe,q);
erre = qerr(:,1:3)*2*1e6;
sig3 = 3*pcov.^(0.5);
sig3(:,1:3)  = sig3(:,1:3)*1e6;
sig3(:,4:6)  = sig3(:,4:6)*3600*180/pi;
sig3(:,7:15) = sig3(:,7:15)*1e6;

%% SMOOTHER
for i = m-1:-1:1                         % Loop through all time steps
 pp_k  = shiftdim(pp_store(i,:,:),1);     % Prior Error Covariance
 pm_k1 = shiftdim(pm_store(i+1,:,:),1);   % Propagated Error Covariance
 phi_k = shiftdim(phi_store(i,:,:),1);
 xe_k  = shiftdim(xp_store(i,:,:),1);
 xe_k1 = shiftdim(xm_store(i+1,:,:),1);
 
 % smoother equations
 gain  = pp_k*phi_k'/(pm_k1);
 ps    = pp_k-gain*(pm_k1-ps)*gain';                % Smoothed Error Covariance
 pscov(i,:) = diag(ps)';                            % Diagonals of Smoothed Error Covariance
 xe_s(i,:)  = xe_k +(gain*(xe_s(i+1,:)'- xe_k1'))'; % Smoothed State Estimates
 
 xee  = [0.5*xe_s(i,1:3) 1];                             % dQuaternions Estimate from Smoothed State Estimates

 % Update Quaternion
 qe11 =  xee(1,4)*qe(i,1)+xee(1,3)*qe(i,2)-xee(1,2)*qe(i,3)+xee(1,1)*qe(i,4);
 qe22 = -xee(1,3)*qe(i,1)+xee(1,4)*qe(i,2)+xee(1,1)*qe(i,3)+xee(1,2)*qe(i,4);
 qe33 =  xee(1,2)*qe(i,1)-xee(1,1)*qe(i,2)+xee(1,4)*qe(i,3)+xee(1,3)*qe(i,4);
 qe44 = -xee(1,1)*qe(i,1)-xee(1,2)*qe(i,2)-xee(1,3)*qe(i,3)+xee(1,4)*qe(i,4);
 qe_s(i,:) = [qe11 qe22 qe33 qe44]./norm([qe11 qe22 qe33 qe44]);
 
end

qerr   = quat_err(qe_s,q);
erre_s = qerr(:,1:3)*2*1e6;
sig3s  = pscov.^(0.5)*3;
sig3s(:,1:3)  = sig3s(:,1:3)*1e6;
sig3s(:,4:6)  = sig3s(:,4:6)*3600*180/pi;
% sig3s(:,7:15) = sig3s(:,7:15)*1e6;


%% Errors (note: tracker boresight is the z-axis)
subplot(311)
plot(t/60,sig3(:,1),'--r',t/60,erre(:,1),'b',t/60,-sig3(:,1),'--r');
set(gca,'Fontsize',12);
ylabel('Roll (mrad)')
grid
axis([0 90 -20 20])
set(gca,'Xtick',[0 15 30 45 60 75 90])
set(gca,'Ytick',[-20 0 20])
subplot(312)
plot(t/60,sig3(:,2),'--r',t/60,erre(:,2),'b',t/60,-sig3(:,2),'--r');
set(gca,'Fontsize',12);
ylabel('Pitch (mrad)')
grid
axis([0 90 -20 20])
set(gca,'Xtick',[0 15 30 45 60 75 90])
set(gca,'Ytick',[-20 0 20])
subplot(313)
plot(t/60,sig3(:,3),'--r',t/60,erre(:,3),'b',t/60,-sig3(:,3),'--r');
set(gca,'Fontsize',12);
ylabel('Yaw (mrad)')
grid
axis([0 90 -20 20])
set(gca,'Xtick',[0 15 30 45 60 75 90])
set(gca,'Ytick',[-20 0 20])
xlabel('Time (Min)')



%% Biases
figure
subplot(311)
plot(t/60,sig3(:,4),'--r',t/60,(xe(:,4)-bias1)*180/pi*3600,'b',t/60,-sig3(:,4),'--r');
set(gca,'Fontsize',12);
ylabel('X (Deg/Hr)')
grid
axis([0 90 -0.02 0.02])
set(gca,'Xtick',[0 15 30 45 60 75 90])
set(gca,'Ytick',[-0.02 0 0.02])
subplot(312)
plot(t/60,sig3(:,5),'--r',t/60,(xe(:,5)-bias2)*180/pi*3600,'b',t/60,-sig3(:,5),'--r');
set(gca,'Fontsize',12);
ylabel('Y (Deg/Hr)')
grid
axis([0 90 -0.02 0.02])
set(gca,'Xtick',[0 15 30 45 60 75 90])
set(gca,'Ytick',[-0.02 0 0.02])
subplot(313)
plot(t/60,sig3(:,6),'--r',t/60,(xe(:,6)-bias3)*180/pi*3600,'b',t/60,-sig3(:,6),'--r');
set(gca,'Fontsize',12);
ylabel('Z (Deg/Hr)')
grid
axis([0 90 -0.02 0.02])
set(gca,'Xtick',[0 15 30 45 60 75 90])
set(gca,'Ytick',[-0.02 0 0.02])
xlabel('Time (Min)')

% Scale Factors
figure
subplot(311)
plot(t/60,sig3(:,7),'--r',t/60,(xe(:,7)-s(1,1))*1e6,'b',t/60,-sig3(:,7),'--r');
set(gca,'Fontsize',12);
ylabel('s1 (mrad)')
grid
axis([0 90 -60 60])
set(gca,'Xtick',[0 15 30 45 60 75 90])
set(gca,'Ytick',[-60 0 60])
subplot(312)
plot(t/60,sig3(:,8),'--r',t/60,(xe(:,8)-s(2,2))*1e6,'b',t/60,-sig3(:,8),'--r');
set(gca,'Fontsize',12);
ylabel('s2 (mrad)')
grid
axis([0 90 -60 60])
set(gca,'Xtick',[0 15 30 45 60 75 90])
set(gca,'Ytick',[-60 0 60])
subplot(313)
plot(t/60,sig3(:,9),'--r',t/60,(xe(:,9)-s(3,3))*1e6,'b',t/60,-sig3(:,9),'--r');
set(gca,'Fontsize',12);
ylabel('s3 (mrad)')
grid
axis([0 90 -60 60])
set(gca,'Xtick',[0 15 30 45 60 75 90])
set(gca,'Ytick',[-60 0 60])
xlabel('Time (Min)')

% Upper Misalignments
figure
subplot(311)
plot(t/60,sig3(:,10),'--r',t/60,(xe(:,10)-s(1,2))*1e6,'b',t/60,-sig3(:,10),'--r');
set(gca,'Fontsize',12);
ylabel('kU1 (mrad)')
grid
axis([0 90 -60 60])
set(gca,'Xtick',[0 15 30 45 60 75 90])
set(gca,'Ytick',[-60 0 60])
subplot(312)
plot(t/60,sig3(:,11),'--r',t/60,(xe(:,11)-s(1,3))*1e6,'b',t/60,-sig3(:,11),'--r');
set(gca,'Fontsize',12);
ylabel('kU2 (mrad)')
grid
axis([0 90 -60 60])
set(gca,'Xtick',[0 15 30 45 60 75 90])
set(gca,'Ytick',[-60 0 60])
subplot(313)
plot(t/60,sig3(:,12),'--r',t/60,(xe(:,12)-s(2,3))*1e6,'b',t/60,-sig3(:,12),'--r');
set(gca,'Fontsize',12);
ylabel('kU3 (mrad)')
grid
axis([0 90 -60 60])
set(gca,'Xtick',[0 15 30 45 60 75 90])
set(gca,'Ytick',[-60 0 60])
xlabel('Time (Min)')

% Lower Misalignments
figure
subplot(311)
plot(t/60,sig3(:,13),'--r',t/60,(xe(:,13)-s(2,1))*1e6,'b',t/60,-sig3(:,13),'--r');
set(gca,'Fontsize',12);
ylabel('kL1 (mrad)')
grid
axis([0 90 -60 60])
set(gca,'Xtick',[0 15 30 45 60 75 90])
set(gca,'Ytick',[-60 0 60])
subplot(312)
plot(t/60,sig3(:,14),'--r',t/60,(xe(:,14)-s(3,1))*1e6,'b',t/60,-sig3(:,14),'--r');
set(gca,'Fontsize',12);
ylabel('kL2 (mrad)')
grid
axis([0 90 -60 60])
set(gca,'Xtick',[0 15 30 45 60 75 90])
set(gca,'Ytick',[-60 0 60])
subplot(313)
plot(t/60,sig3(:,15),'--r',t/60,(xe(:,15)-s(3,2))*1e6,'b',t/60,-sig3(:,15),'--r');
set(gca,'Fontsize',12);
ylabel('kL3 (mrad)')
grid
axis([0 90 -60 60])
set(gca,'Xtick',[0 15 30 45 60 75 90])
set(gca,'Ytick',[-60 0 60])
xlabel('Time (Min)')

return
%% SMOOTHER
disp(' Press any key to continue')
pause

% Smoother
subplot(311)
plot(t/60,sig3s(:,1),'--r',t/60,erre_s(:,1),'b',t/60,-sig3s(:,1),'--r');
set(gca,'Fontsize',12);
ylabel('Roll (mrad)')
grid
axis([0 90 -15 15])
set(gca,'Xtick',[0 15 30 45 60 75 90])
set(gca,'Ytick',[-15 0 15])
subplot(312)
plot(t/60,sig3s(:,2),'--r',t/60,erre_s(:,2),'b',t/60,-sig3s(:,2),'--r');
set(gca,'Fontsize',12);
ylabel('Pitch (mrad)')
grid
axis([0 90 -15 15])
set(gca,'Xtick',[0 15 30 45 60 75 90])
set(gca,'Ytick',[-15 0 15])
subplot(313)
plot(t/60,sig3s(:,3),'--r',t/60,erre_s(:,3),'b',t/60,-sig3s(:,3),'--r');
set(gca,'Fontsize',12);
ylabel('Yaw (mrad)')
grid
axis([0 90 -15 15])
set(gca,'Xtick',[0 15 30 45 60 75 90])
set(gca,'Ytick',[-15 0 15])
xlabel('Time (Min)')
%axis([0 90 -150 150])
%set(gca,'Xtick',[0 15 30 45 60 75 90])
%set(gca,'Ytick',[-150 0 150])
%xlabel('Time (Min)')

disp(' Press any key to continue')
pause

% Biases
subplot(311)
plot(t/60,sig3s(:,4),'--r',t/60,(xe_s(:,4)-bias1)*180/pi*3600,'b',t/60,-sig3s(:,4),'--r');
set(gca,'Fontsize',12);
ylabel('X (Deg/Hr)')
grid
axis([0 90 -0.01 0.01])
set(gca,'Xtick',[0 15 30 45 60 75 90])
set(gca,'Ytick',[-0.01 0 0.01])
subplot(312)
plot(t/60,sig3s(:,5),'--r',t/60,(xe_s(:,5)-bias2)*180/pi*3600,'b',t/60,-sig3s(:,5),'--r');
set(gca,'Fontsize',12);
ylabel('Y (Deg/Hr)')
grid
axis([0 90 -0.01 0.01])
set(gca,'Xtick',[0 15 30 45 60 75 90])
set(gca,'Ytick',[-0.01 0 0.01])
subplot(313)
plot(t/60,sig3s(:,6),'--r',t/60,(xe_s(:,6)-bias3)*180/pi*3600,'b',t/60,-sig3s(:,6),'--r');
set(gca,'Fontsize',12);
ylabel('Z (Deg/Hr)')
grid
axis([0 90 -0.01 0.01])
set(gca,'Xtick',[0 15 30 45 60 75 90])
set(gca,'Ytick',[-0.01 0 0.01])
xlabel('Time (Min)')

disp(' Press any key to continue')
pause

% Scale Factors
subplot(311)
plot(t/60,sig3s(:,7),'--r',t/60,(xe_s(:,7)-s(1,1))*1e6,'b',t/60,-sig3s(:,7),'--r');
set(gca,'Fontsize',12);
ylabel('s1 (mrad)')
grid
axis([0 90 -30 30])
set(gca,'Xtick',[0 15 30 45 60 75 90])
set(gca,'Ytick',[-30 0 30])
subplot(312)
plot(t/60,sig3s(:,8),'--r',t/60,(xe_s(:,8)-s(2,2))*1e6,'b',t/60,-sig3s(:,8),'--r');
set(gca,'Fontsize',12);
ylabel('s2 (mrad)')
grid
axis([0 90 -30 30])
set(gca,'Xtick',[0 15 30 45 60 75 90])
set(gca,'Ytick',[-30 0 30])
subplot(313)
plot(t/60,sig3s(:,9),'--r',t/60,(xe_s(:,9)-s(3,3))*1e6,'b',t/60,-sig3s(:,9),'--r');
set(gca,'Fontsize',12);
ylabel('s3 (mrad)')
grid
axis([0 90 -30 30])
set(gca,'Xtick',[0 15 30 45 60 75 90])
set(gca,'Ytick',[-30 0 30])
xlabel('Time (Min)')

disp(' Press any key to continue')
pause

% Upper Misalignments
subplot(311)
plot(t/60,sig3s(:,10),'--r',t/60,(xe_s(:,10)-s(1,2))*1e6,'b',t/60,-sig3s(:,10),'--r');
set(gca,'Fontsize',12);
ylabel('kU1 (mrad)')
grid
axis([0 90 -30 30])
set(gca,'Xtick',[0 15 30 45 60 75 90])
set(gca,'Ytick',[-30 0 30])
subplot(312)
plot(t/60,sig3s(:,11),'--r',t/60,(xe_s(:,11)-s(1,3))*1e6,'b',t/60,-sig3s(:,11),'--r');
set(gca,'Fontsize',12);
ylabel('kU2 (mrad)')
grid
axis([0 90 -30 30])
set(gca,'Xtick',[0 15 30 45 60 75 90])
set(gca,'Ytick',[-30 0 30])
subplot(313)
plot(t/60,sig3s(:,12),'--r',t/60,(xe_s(:,12)-s(2,3))*1e6,'b',t/60,-sig3s(:,12),'--r');
set(gca,'Fontsize',12);
ylabel('kU3 (mrad)')
grid
axis([0 90 -30 30])
set(gca,'Xtick',[0 15 30 45 60 75 90])
set(gca,'Ytick',[-30 0 30])
xlabel('Time (Min)')

disp(' Press any key to continue')
pause

% Lower Misalignments
subplot(311)
plot(t/60,sig3s(:,13),'--r',t/60,(xe_s(:,13)-s(2,1))*1e6,'b',t/60,-sig3s(:,13),'--r');
set(gca,'Fontsize',12);
ylabel('kL1 (mrad)')
grid
axis([0 90 -30 30])
set(gca,'Xtick',[0 15 30 45 60 75 90])
set(gca,'Ytick',[-30 0 30])
subplot(312)
plot(t/60,sig3s(:,14),'--r',t/60,(xe_s(:,14)-s(3,1))*1e6,'b',t/60,-sig3s(:,14),'--r');
set(gca,'Fontsize',12);
ylabel('kL2 (mrad)')
grid
axis([0 90 -30 30])
set(gca,'Xtick',[0 15 30 45 60 75 90])
set(gca,'Ytick',[-30 0 30])
subplot(313)
plot(t/60,sig3s(:,15),'--r',t/60,(xe_s(:,15)-s(3,2))*1e6,'b',t/60,-sig3s(:,15),'--r');
set(gca,'Fontsize',12);
ylabel('kL3 (mrad)')
grid
axis([0 90 -30 30])
set(gca,'Xtick',[0 15 30 45 60 75 90])
set(gca,'Ytick',[-30 0 30])
xlabel('Time (Min)')