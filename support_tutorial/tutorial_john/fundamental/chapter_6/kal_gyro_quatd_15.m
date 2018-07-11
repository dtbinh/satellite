function [qk1,xest,p_cov,pp_store,pm_store,phi_store,xp_store,xm_store]=kal_gyro_quatd(qm,wgm,R,sigu,sigv,poa,pog,pos,poku,pokl,dt,x0)
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
x0 = x0(:)';
m  = length(qm);

qcov = blkdiag(sigv^2*eye(3),sigu^2*eye(3));

% Pre-Allocate Space
qk1       = zeros(m,4);  % Estimated Quaternion
xest      = zeros(m,15); 
p_cov     = zeros(m,15);
pp_store  = zeros(m,15,15);
pm_store  = zeros(m,15,15);
phi_store = zeros(m,15,15);
xp_store  = zeros(m,1,15);
xm_store  = zeros(m,1,15);

% Initial Bias, Quaternion Estimate
qk1(1,:)  = x0(1,1:4);           % Initial Quaternion Estimate
xest(1,:) = [0 0 0 x0(1,5:16)];  % Initial State Estimate

% Determine Initial Covariance and Discrete State Covariance 
Pk1 = blkdiag(poa,pog,pos,poku,pokl);
p_cov(1,:) = diag(Pk1)';

pm_store(1,:,:) = Pk1;

%% Main Loop
for i = 1:m-1
    % Measurement
    qmm = quat_mult(qk1(i,:),[qm(i,1) qm(i,2) qm(i,3) -qm(i,4)]);    
    inn = 2*[qmm(1) qmm(2) qmm(3)]';   % Innovation of dTheta
    
    H = [eye(3) zeros(3,12)];   % Sensitivity Matrix
    K = Pk1*H'/(H*Pk1*H'+R);    % Kalman Gain
    Pk1 = (eye(15)-K*H)*Pk1; 
    
    xest(i,:) = xest(i,:)+(K*inn)';
    
    pp_store(i,:,:) = Pk1;
    xp_store(i,:,:) = xest(i,:);
    
    % Update
    xe       = [0.5*xest(i,1:3) 1];
    qk1(i,:) = quat_mult(xe,qk1(i,:));
    qk1(i,:) =[qk1(i,1) qk1(i,2) qk1(i,3) qk1(i,4)]./norm([qk1(i,1) qk1(i,2) qk1(i,3) qk1(i,4)]);
    
    
    sest = [xest(i,7) xest(i,10) xest(i,11);
            xest(i,13) xest(i,8) xest(i,12);
            xest(i,14) xest(i,15) xest(i,9)];    
    we = (eye(3)-sest)*(wgm(i,:)'-xest(i,4:6)');
    
    % Propagate Covariance
    we_nos = wgm(i,:)'-xest(i,4:6)';
    uhat   = [we_nos(2) we_nos(3)    0;
                  0         0     we_nos(3);
                  0         0        0];
              
    lhat   = [    0         0        0;
              we_nos(1)     0        0;
                  0      we_nos(1) we_nos(2)];
              
           
    fmat = [-smtrx(we) -(eye(3)-sest) -diag(we_nos)   -uhat     -lhat;
             zeros(3)      zeros(3)      zeros(3)    zeros(3)  zeros(3);
             zeros(3)      zeros(3)      zeros(3)    zeros(3)  zeros(3);
             zeros(3)      zeros(3)      zeros(3)    zeros(3)  zeros(3);
             zeros(3)      zeros(3)      zeros(3)    zeros(3)  zeros(3)];
    gmat = [-(eye(3)-sest) zeros(3);
               zeros(3)     eye(3);
               zeros(3)    zeros(3);
               zeros(3)    zeros(3);
               zeros(3)    zeros(3)];

    phik = eye(15)+dt*fmat;
    
    phi_store(i,:,:) = phik;
    
    qcovd = dt*gmat*qcov*gmat';
    Pk1 = phik*Pk1*phik'+qcovd; 

    % Estimated State Propagation
    psik  =  sin(1/2*norm(we)*dt)/norm(we)*we;
    omega = [cos(1/2*norm(we)*dt)*eye(3)-smtrx(psik)       psik;
                        -psik'                       cos(1/2*norm(we)*dt) ];
    qk1(i+1,1:4)= omega*qk1(i,1:4)';

    
    pm_store(i+1,:,:) = Pk1;        % Covariance Matrix  
    p_cov(i+1,:)      = diag(Pk1)'; % Diagonals of Covariance Matrix
    xest(i+1,:)       = xest(i,:);
    xest(i+1,1:3)     = [0 0 0];
    xm_store(i+1,:,:) = xest(i+1,:);
end