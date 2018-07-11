% This example shows a simulation using a typical star camera 
% with gyro measurements to determine the attitude of a rotating 
% spacecraft using an extended Kalman filter. This program assumes
% quaternion measurements and also runs a smoother to determine 
% the gyro calibration parameters.

% Fundamentals of Spacecraft Attitude Determination and Control by Markley and Crassidis
% Example 6.1

% Written by John L. Crassidis 9/13

% Other Required Routines: quat_err.m, quat_mult.m

% Time 
dt=1;tf=90*60;t=[0:dt:tf]';m=length(t);

% True Rate Quaternion
w=0.1*pi/180*[sin(0.01*t) sin(0.0085*t) cos(0.0085*t)];
q=zeros(m,4);q(1,:)=1/sqrt(2)*[1 0 0 1];
for i=1:m-1
 ww=norm(w(i,:));
 co=cos(0.5*ww*dt);
 si=sin(0.5*ww*dt);
 n1=w(i,1)/ww;n2=w(i,2)/ww;n3=w(i,3)/ww;
 qw1=n1*si;qw2=n2*si;qw3=n3*si;qw4=co;
 om=[qw4  qw3 -qw2 qw1;-qw3  qw4  qw1 qw2;qw2 -qw1  qw4 qw3;-qw1 -qw2 -qw3 qw4];
 q(i+1,:)=(om*q(i,:)')';
end

% Quaternion Measurements Directly 
sig_tracker=6/3600*pi/180;
noise=0.5*sig_tracker*randn(m,3);
qm=quat_mult(q,[noise ones(m,1)]);
qmnorm=(qm(:,1).^2+qm(:,2).^2+qm(:,3).^2+qm(:,4).^2).^(0.5);
qm(:,1)=qm(:,1)./qmnorm;qm(:,2)=qm(:,2)./qmnorm;qm(:,3)=qm(:,3)./qmnorm;qm(:,4)=qm(:,4)./qmnorm;


% True Scale Factors and Misalignments
s=1e-6*[1500 1000 1500;500 1000 2000;1000 1500 1500];

% Get True Rates (note: bias is simulated in discrete-time)
% From Reynolds, R., "Maximum Likelihood Estimation of Stability Parameters for the Standard 
% Gyroscopic Error Model," Flight Mechanics Symposium, NASA-Goddard Space Flight Center, 
% Greenbelt, MD, Oct. 2003, NASA CP-2003-212246, paper #42.
wtrue=w;
sigu=sqrt(10)*1e-10;
sigv=sqrt(10)*1e-7;
num_g=dt*[1 1];den_g=2*[1 -1];
[phi_g,gam_g,c_g,d_g]=tf2ss(num_g,den_g);
bias1=dlsim(phi_g,gam_g,c_g,d_g,sigu/sqrt(dt)*randn(m,1),0.1*pi/180/3600/dt);
bias2=dlsim(phi_g,gam_g,c_g,d_g,sigu/sqrt(dt)*randn(m,1),0.1*pi/180/3600/dt);
bias3=dlsim(phi_g,gam_g,c_g,d_g,sigu/sqrt(dt)*randn(m,1),0.1*pi/180/3600/dt);
bias=[bias1 bias2 bias3];
wgm=((eye(3)+s)*wtrue')'+sqrt(sigv^2/dt+1/12*sigu^2*dt)*randn(m,3)+bias;

% Initial Conditions and Covariances
x0=[q(1,:)';zeros(12,1)];
poa=(0.1*pi/180)^2*eye(3);
pog=(0.2*pi/180/3600)^2*eye(3);
pos=(2000*1e-6/3)^2*eye(3);
poku=(2000*1e-6/3)^2*eye(3);
pokl=(2000*1e-6/3)^2*eye(3);

% EKF 
[qe,xe,pcov,pp_store,pm_store,phi_store,xp_store,xm_store]=kal_gyro_quatd(qm,wgm,sig_tracker^2*eye(3),sigu,sigv,poa,pog,pos,poku,pokl,dt,x0);

% Smoother
ps=shiftdim(pm_store(m,:,:),1);pscov=zeros(m,15);pscov(m,:)=diag(ps)';
xe_s=zeros(m,15);xe_s(m,:)=shiftdim(xm_store(m,:,:),1);
qe_s=zeros(m,4);qe_s(m,:)=qe(m,:);

for i=m-1:-1:1
 pp_k=shiftdim(pp_store(i,:,:),1);pm_k1=shiftdim(pm_store(i+1,:,:),1);
 gain=pp_k*shiftdim(phi_store(i,:,:),1)'*inv(pm_k1);
 ps=pp_k-gain*(pm_k1-ps)*gain';
 pscov(i,:)=diag(ps)';
 xe_s(i,:)=shiftdim(xp_store(i,:,:),1)+(gain*(xe_s(i+1,:)'-shiftdim(xm_store(i+1,:,:),1)'))';
 
 xee=0.5*xe_s(i,1:3);
 qe11=qe(i,1)+xee(1,3).*qe(i,2)-xee(1,2).*qe(i,3)+xee(1,1).*qe(i,4);
 qe22=-xee(1,3).*qe(i,1)+qe(i,2)+xee(1,1).*qe(i,3)+xee(1,2).*qe(i,4);
 qe33=xee(1,2).*qe(i,1)-xee(1,1).*qe(i,2)+qe(i,3)+xee(1,3).*qe(i,4);
 qe44=-xee(1,1).*qe(i,1)-xee(1,2).*qe(i,2)-xee(1,3).*qe(i,3)+qe(i,4);
 qe_s(i,:)=[qe11 qe22 qe33 qe44]./norm([qe11 qe22 qe33 qe44]);
end


% Compute Attitude Errors and 3-Sigma Outlier
qerr=quat_err(qe,q);erre=qerr(:,1:3)*2*1e6;
sig3=pcov.^(0.5)*3;
sig3(:,1:3)=sig3(:,1:3)*1e6;
sig3(:,4:6)=sig3(:,4:6)*3600*180/pi;
sig3(:,7:15)=sig3(:,7:15)*1e6;
qerr=quat_err(qe_s,q);erre_s=qerr(:,1:3)*2*1e6;
sig3s=pscov.^(0.5)*3;
sig3s(:,1:3)=sig3s(:,1:3)*1e6;
sig3s(:,4:6)=sig3s(:,4:6)*3600*180/pi;
sig3s(:,7:15)=sig3s(:,7:15)*1e6;


% Errors (note: tracker boresight is the z-axis)
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

disp(' Press any key to continue')
pause

% Biases
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

disp(' Press any key to continue')
pause

% Scale Factors
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

disp(' Press any key to continue')
pause

% Upper Misalignments
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

disp(' Press any key to continue')
pause

% Lower Misalignments
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