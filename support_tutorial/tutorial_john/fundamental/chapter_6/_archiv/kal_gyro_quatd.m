function [qe,xest,p_cov,pp_store,pm_store,phi_store,xp_store,xm_store]=kal_gyro_quatd(qm,wgm,r,sigu,sigv,poa,pog,pos,poku,pokl,dt,x0);
%function [qe,xest,p_cov]=kal_gyro_quatd(qm,wgm,r,sigu,sigv,poa,pog,pos,poku,pokl,dt,x0);
%
% This program determines the attitude of a satellite using a full
% Kalman filter (see Lefferts, Markley, and Shuster, JGCD Sept.-Oct. '82).
% This algorithm uses the discrete propagated error covariance.
% This version uses multiple sensors. Units are in radians and seconds.
%
% The inputs are:
%     im = inertial measurements [mx(3*s)], s = number of sensors
%     bm = body measurements  [mx(3*s)]
%    wgm = gyro measurements [mx3]
%      r = measurement covariance [(3*s)x(3*s)]
%   sigu = gyro bias noise standard deviation [3x3]
%   sigv = gyro noise standard deviation [3x3]
%    poa = initial error covariance of attitude [3x3]
%    pog = initial error covariance of gyro bias [3x3]
%     av = 1 for sensor available, 0 for not  [mxs]
%     dt = sampling interval
%     x0 = initial estimates of quaternions and biases ([q0 b0]), [1x7]
%
% The outputs are:
%     qe = estimated quaternions [mx4]
%     be = estimated gyro biases [mx3]
%  p_cov = diagonal covariances [mx6]

% Version 2, Written by John L Crassidis 8/22/94

% Constants and Conversions
x0=x0(:)';
m=length(qm);
i500=0;

qcov=blkdiag(sigv^2*eye(3),sigu^2*eye(3));

% Sensor to Body Measurements
%for mm=1:sen,
% bm(:,mm*3-2:mm*3)=(al(:,mm*3-2:mm*3)'*bm(:,mm*3-2:mm*3)')';
%end

% Pre-Allocate Space
qe=zeros(m,4);
xest=zeros(m,15);
p_cov=zeros(m,15);
pp_store=zeros(m,15,15);
pm_store=zeros(m,15,15);
phi_store=zeros(m,15,15);
xp_store=zeros(m,1,15);
xm_store=zeros(m,1,15);

% Initial Bias, Quaternion Estimate
qe(1,:)=x0(1,1:4);
xest(1,:)=[0 0 0 x0(1,5:16)];

% Determine Initial Covariance and Discrete State Covariance 
p=blkdiag(poa,pog,pos,poku,pokl);
p_cov(1,:)=diag(p)';
pm_store(1,:,:)=p;

% Main Loop
for i=1:m-1,

% Display When Every 500th Point Is Reached
if (i500==500), 
 disp(sprintf('      Filter has reached point %5i',i-1))
 i500=0;
end
i500=i500+1;

qmm1=-qm(i,4)*qe(i,1)-qm(i,3)*qe(i,2)+qm(i,2)*qe(i,3)+qm(i,1)*qe(i,4);
qmm2= qm(i,3)*qe(i,1)-qm(i,4)*qe(i,2)-qm(i,1)*qe(i,3)+qm(i,2)*qe(i,4);
qmm3=-qm(i,2)*qe(i,1)+qm(i,1)*qe(i,2)-qm(i,4)*qe(i,3)+qm(i,3)*qe(i,4);
z=2*[qmm1 qmm2 qmm3]';
h=[eye(3) zeros(3,12)];
k=p*h'*inv(h*p*h'+r);
%p=(eye(15)-k*h)*p;
p=(eye(15)-k*h)*p*(eye(15)-k*h)'+k*r*k';
xest(i,:)=xest(i,:)+(k*z)';

pp_store(i,:,:)=p;
xp_store(i,:,:)=xest(i,:);

% Save Output
xe=0.5*xest(i,1:3);
qe11=qe(i,1)+xe(1,3).*qe(i,2)-xe(1,2).*qe(i,3)+xe(1,1).*qe(i,4);
qe22=-xe(1,3).*qe(i,1)+qe(i,2)+xe(1,1).*qe(i,3)+xe(1,2).*qe(i,4);
qe33=xe(1,2).*qe(i,1)-xe(1,1).*qe(i,2)+qe(i,3)+xe(1,3).*qe(i,4);
qe44=-xe(1,1).*qe(i,1)-xe(1,2).*qe(i,2)-xe(1,3).*qe(i,3)+qe(i,4);
qe(i,:)=[qe11 qe22 qe33 qe44]./norm([qe11 qe22 qe33 qe44]);

% Propagate Covariance
sest=[xest(i,7) xest(i,10) xest(i,11);xest(i,13) xest(i,8) xest(i,12);xest(i,14) xest(i,15) xest(i,9)];
we=(eye(3)-sest)*(wgm(i,:)'-xest(i,4:6)');
we_nos=wgm(i,:)'-xest(i,4:6)';
uhat=[we_nos(2) we_nos(3) 0;0 0 we_nos(3);0 0 0];
lhat=[0 0 0;we_nos(1) 0 0;0 we_nos(1) we_nos(2)];
wec=[0   -we(3)   we(2)
    we(3)  0     -we(1)
   -we(2) we(1)    0];
fmat=[-wec -(eye(3)-sest) -diag(we_nos) -uhat -lhat;zeros(12,15)];
gmat=[-(eye(3)-sest) zeros(3);zeros(3) eye(3);zeros(9,6)];
% biga=[-fmat gmat*qcov*gmat';zeros(15) fmat']*dt;
% bigb=expm(biga);
% phi=bigb(16:30,16:30)';
% qcovd=phi*bigb(1:15,16:30);

phi=(eye(15)+dt*fmat);
phi_store(i,:,:)=phi;
qcovd=dt*gmat*qcov*gmat';

% Propagate State
w=norm(we);
co=cos(0.5*w*dt);
si=sin(0.5*w*dt);
n1=we(1)/w;n2=we(2)/w;n3=we(3)/w;
qw1=n1*si;qw2=n2*si;qw3=n3*si;qw4=co;
om=[qw4  qw3 -qw2 qw1;-qw3  qw4  qw1 qw2;qw2 -qw1  qw4 qw3;-qw1 -qw2 -qw3 qw4];
qe(i+1,1:4)=(om*qe(i,1:4)')';

% Propagate Covariance
p=phi*p*phi'+qcovd; 
pm_store(i+1,:,:)=p;
p_cov(i+1,:)=diag(p)';
xest(i+1,:)=xest(i,:);
xest(i+1,1:3)=[0 0 0];
xm_store(i+1,:,:)=xest(i+1,:);

% Display Error If Covariance Is Going Negative
if (length(find(p_cov(i,:)'>0))<15),
 disp(sprintf('error covariance is not positive, point %1i',i))
end

end