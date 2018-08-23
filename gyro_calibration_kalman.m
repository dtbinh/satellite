close all
clear all
clc

%% INITIALISATION
fprintf('---------GYRO CALIBRATION KALMAN SIMULATION--------\n');
dt   = 1;          % [s] Model Sampling and Gyro Sampling Time 
tdur = 90*60;         % [s] Time Duration
N    = tdur/dt;       % [] Number of Samples

tout = 0:dt:tdur;
m    = length(tout);

sig_v = sqrt(10)*1e-7;    % [rad/s^0.5] Angular Random Walk
sig_u = sqrt(10)*1e-10;   % [rad/s^1.5] Rate Random Walk
sig_s = 0;
sig_U = 0;
sig_L = 0;

sig_st = 6/3600*pi/180;                 % [rad] Sig of Star Tracker Measurement

bias_0 = [10; 10; 10]/180*pi/3600; % [rad/s] Initial Gyro Bias

bias = bias_0;

S     = [1500 1000 1500;
          500 1000 2000;
         1000 1500 1500]/1e6;    % [ppm] Gyro Scale Factors

P_a_0 = (1*pi/180)^2*eye(3);     % Initial Covariance - Attitude
P_b_0 = (2*pi/180/3600)^2*eye(3);   % Initial Covariance - Gyro Bias
P_s_0 = (20000*1e-6/3)^2*eye(3); % Initial Covariance - Scale Factor
P_u_0 = (20000*1e-6/3)^2*eye(3); % Initial Covariance - Upper Misalignment
P_l_0 = (20000*1e-6/3)^2*eye(3); % Initial Covariance - Lower Misalignment

w_B_BI_0 = [0.0;0.0;0.0/180*pi];  % Initial Angular Velocity
q_B_I_0  = 1/sqrt(2)*[1; 0;0; 1]; % Initial Quaternion
omega_m  = [0;0;0];

%% GENERATING SATELLITE MOTION
%True Rate Quaternion
w      = 0.1*pi/180*[sin(0.01*tout);sin(0.0085*tout);cos(0.0085*tout)];
q      = zeros(4,N);          % Prepare
q(:,1) = 1/sqrt(2)*[1;0;0;1]; % Initialise

% Obtaining the true quaternion
for i=1:N
 psik  =  sin(1/2*norm(w(:,i))*dt)/norm(w(:,i))*w(:,i);
 omega = [cos(1/2*norm(w(:,i))*dt)*eye(3)-smtrx(psik)       psik;
                    -psik'                       cos(1/2*norm(w(:,i))*dt) ];
   
 q(:,i+1) = omega*q(:,i);
end

w_B_BI = w;
q_B_I  = q;


%% SENSOR MEASUREMENT
for i=1:length(tout)
    bias(:,i+1)   = bias(:,i)+sig_u*dt^0.5*[randn(1,1);randn(1,1);randn(1,1)];
    bias_m(:,i)   = 0.5*(bias(:,i+1)+bias(:,i));
    w_B_BI_m(:,i) = (eye(3)+S)*w_B_BI(:,i)+sqrt(sig_v^2/dt+1/12*sig_u^2*dt)*[randn(1,1);randn(1,1);randn(1,1)]+bias_m(:,i);

    q_B_I_m(:,i)  = qmul(q_B_I(:,i),[0.5*sig_st*[randn(1,1);randn(1,1);randn(1,1)];1]);
    q_B_I_m(:,i)  = qnorm(q_B_I_m(:,i));
end


%% EXTENDED KALMAN FILTER
% Initialise
	qk1  = q_B_I(:,1);  % Initial Quaternion (xyzw) for internal variable
    bk1  = [0.0;0.0;0.0]; % Initial Bias Value 
    sk1  = [0.0;0.0;0.0]; % Initial Bias Value 
    uk1  = [0.0;0.0;0.0]; % Initial Bias Value 
    lk1  = [0.0;0.0;0.0]; % Initial Bias Value
    wk1  = [0.0;0.0;0.0]; % Initial Angular Velocity
    
    xest = zeros(15,1);   % State Estimate

	Pk1  = blkdiag(P_a_0,P_b_0,P_s_0, P_u_0, P_l_0);
	qcov = blkdiag(sig_v^2*eye(3),sig_u^2*eye(3));
	R  = sig_st^2*eye(3);
    
    q_B_I_f(:,1)  = qk1; % Output Initial Quaternion (xyzw) for internal variable
    bias_f(:,1)   = bk1; % Output Initial Bias Value
    s_f(:,1)      = sk1; % Output Initial s misalignment value        
    u_f(:,1)      = uk1; % Output Initial u misalignment value   
    l_f(:,1)      = lk1; % Output Initial l misalignment value    
    w_B_BI_f(:,1) = wk1; % Initial Value from measured gyro
    Pk_f(:,:,1)   = Pk1;
    
% Kalman Loop
for i=2:length(tout)-1
  
    % Measurement 
        qmm = qmul(qk1,[q_B_I_m(1,i);q_B_I_m(2,i);q_B_I_m(3,i);-q_B_I_m(4,i)]);
        inn = 2*[qmm(1);qmm(2);qmm(3)];
  
        H    = [eye(3) zeros(3,12)];     % Sensitivity Matrix
        K    = Pk1*H'/(H*Pk1*H' + R);    % Nx3 Kalman Gain  
 
        Pk1  = (eye(15) - K*H)*Pk1;      % NxN  

        xest = xest + K*inn;             % State Update
        
      
    % Update 
        dqe  = [0.5*xest(1:3);1];     % Error Estimate of Quaternion      
        qk1  = qmul(dqe,qk1);        % Update Quaternion by Error Quaternion
        qk1  = qk1./norm(qk1);        % Normalisation of Quaternion      
       
        bk1  = bk1 + xest(4:6,1);
        sk1  = sk1 + xest(7:9,1);
        uk1  = uk1 + xest(10:12,1);
        lk1  = lk1 + xest(13:15,1);
        
        Sest = [sk1(1,1) uk1(1,1) uk1(2,1);
                lk1(1,1) sk1(2,1) uk1(3,1);
                lk1(2,1) lk1(3,1) sk1(3,1)];    
        wk1  = (eye(3)-Sest)*(w_B_BI_m(:,i)-bk1); % Angular Velocity Update
     
    
       
    % Estimated Covariance Propagation
    
        we_nos = w_B_BI_m(:,i)-bk1;
        uhat   = [we_nos(2,1) we_nos(3,1)    0;
                      0         0     we_nos(3,1);
                      0         0        0];
              
        lhat   = [    0         0        0;
                  we_nos(1,1)     0        0;
                      0      we_nos(1,1) we_nos(2,1)];
                  
        F = [-smtrx(wk1) -(eye(3)-Sest) -diag(we_nos)   -uhat     -lhat;
             zeros(3)      zeros(3)      zeros(3)    zeros(3)  zeros(3);
             zeros(3)      zeros(3)      zeros(3)    zeros(3)  zeros(3);
             zeros(3)      zeros(3)      zeros(3)    zeros(3)  zeros(3);
             zeros(3)      zeros(3)      zeros(3)    zeros(3)  zeros(3)];
    
         
        G = [-(eye(3)-Sest) zeros(3);
               zeros(3)     eye(3);
               zeros(3)    zeros(3);
               zeros(3)    zeros(3);
               zeros(3)    zeros(3)];

        phik = eye(15) + F*dt;
       
        qcovd = dt*G*qcov*G';
        Pk1 = phik*Pk1*phik'+ qcovd;  
        
    % Estimated State Propagation
        psik  =  sin(1/2*norm(wk1)*dt)/norm(wk1)*wk1;
        omega = [cos(1/2*norm(wk1)*dt)*eye(3)-smtrx(psik)       psik;
                        -psik'                       cos(1/2*norm(wk1)*dt) ];
        qk1   = omega*qk1;
    
    % Output
        q_B_I_f(:,i+1)  = qk1;            % Initial Quaternion (xyzw) for internal variable
        bias_f(:,i+1)   = bk1;            % Initial Bias Value from Lab Calibration
        s_f(:,i+1)      = sk1;
        u_f(:,i+1)      = uk1;
        l_f(:,i+1)      = lk1;
        w_B_BI_f(:,i+1) = wk1;            % Initial Value from measured gyro
        Pk_f(:,:,i+1)   = Pk1;
        p_cov(:,i+1)    = diag(Pk1)';
        xest = zeros(15,1);                % Reset 
end

%% POST PROCESSING
% Compute Attitude Errors and 3-Sigma Outlier
qerr = quat_err(q_B_I_f,q_B_I);
erre = qerr(1:3,:)*2*1e6;
sig3 = 3*p_cov.^(0.5);
sig3(1:3,:)  = sig3(1:3,:)*1e6;
sig3(4:6,:)  = sig3(4:6,:)*3600*180/pi;  % Convert from [rad/s] to [degree/h]
sig3(7:15,:) = sig3(7:15,:)*1e6;         % Convert from [rad] to [mrad]

%% ERROR PLOTS
figure
subplot(311)
plot(tout/60,sig3(1,:),'--r',tout/60,erre(1,:),'b',tout/60,-sig3(1,:),'--r');
ylabel('Roll [mrad]')
grid
axis([-inf inf -20 20])
subplot(312)
plot(tout/60,sig3(2,:),'--r',tout/60,erre(2,:),'b',tout/60,-sig3(2,:),'--r')
ylabel('Pitch [mrad]')
grid
axis([-inf inf -20 20])
subplot(313)
plot(tout/60,sig3(3,:),'--r',tout/60,erre(3,:),'b',tout/60,-sig3(3,:),'--r');
ylabel('Yaw [mrad]')
grid
axis([-inf inf -20 20])
xlabel('Time [min]')
 
%% Biases
figure
subplot(311)
plot(tout/60,sig3(4,:),'--r',tout/60,(bias_f(1,:)-bias_m(1,:))*180/pi*3600,'b',tout/60,-sig3(4,:),'--r');
ylabel('X [deg/h]')
grid
axis([-inf inf -0.02 0.02])
subplot(312)
plot(tout/60,sig3(5,:),'--r',tout/60,(bias_f(2,:)-bias_m(2,:))*180/pi*3600,'b',tout/60,-sig3(5,:),'--r');
ylabel('Y [deg/h]')
grid
axis([-inf inf -0.02 0.02])
subplot(313)
plot(tout/60,sig3(6,:),'--r',tout/60,(bias_f(3,:)-bias_m(3,:))*180/pi*3600,'b',tout/60,-sig3(6,:),'--r');
ylabel('Z [deg/h]')
grid
axis([-inf inf -0.02 0.02])
xlabel('Time [min]')

%% Scale Factors

figure
subplot(311)
plot(tout/60,sig3(7,:),'--r',tout/60,(s_f(1,:)-S(1,1))*1e6,'b',tout/60,-sig3(7,:),'--r');
set(gca,'Fontsize',12);
ylabel('s1 (mrad)')
grid
axis([0 90 -60 60])
set(gca,'Xtick',[0 15 30 45 60 75 90])
set(gca,'Ytick',[-60 0 60])
subplot(312)
plot(tout/60,sig3(8,:),'--r',tout/60,(s_f(2,:)-S(2,2))*1e6,'b',tout/60,-sig3(8,:),'--r');
set(gca,'Fontsize',12);
ylabel('s2 (mrad)')
grid
axis([0 90 -60 60])
set(gca,'Xtick',[0 15 30 45 60 75 90])
set(gca,'Ytick',[-60 0 60])
subplot(313)
plot(tout/60,sig3(9,:),'--r',tout/60,(s_f(3,:)-S(3,3))*1e6,'b',tout/60,-sig3(9,:),'--r');
set(gca,'Fontsize',12);
ylabel('s3 (mrad)')
grid
axis([0 90 -60 60])
set(gca,'Xtick',[0 15 30 45 60 75 90])
set(gca,'Ytick',[-60 0 60])
xlabel('Time (Min)')

%% Upper Misalignments
figure
subplot(311)
plot(tout/60,sig3(10,:),'--r',tout/60,(u_f(1,:)-S(1,2))*1e6,'b',tout/60,-sig3(10,:),'--r');
set(gca,'Fontsize',12);
ylabel('kU1 (mrad)')
grid
axis([0 90 -60 60])
set(gca,'Xtick',[0 15 30 45 60 75 90])
set(gca,'Ytick',[-60 0 60])
subplot(312)
plot(tout/60,sig3(11,:),'--r',tout/60,(u_f(2,:)-S(1,3))*1e6,'b',tout/60,-sig3(11,:),'--r');
set(gca,'Fontsize',12);
ylabel('kU2 (mrad)')
grid
axis([0 90 -60 60])
set(gca,'Xtick',[0 15 30 45 60 75 90])
set(gca,'Ytick',[-60 0 60])
subplot(313)
plot(tout/60,sig3(12,:),'--r',tout/60,(u_f(3,:)-S(2,3))*1e6,'b',tout/60,-sig3(12,:),'--r');
set(gca,'Fontsize',12);
ylabel('kU3 (mrad)')
grid
axis([0 90 -60 60])
set(gca,'Xtick',[0 15 30 45 60 75 90])
set(gca,'Ytick',[-60 0 60])
xlabel('Time (Min)')

%% Lower Misalignments
figure
subplot(311)
plot(tout/60,sig3(13,:),'--r',tout/60,(l_f(1,:)-S(2,1))*1e6,'b',tout/60,-sig3(13,:),'--r');
set(gca,'Fontsize',12);
ylabel('kL1 (mrad)')
grid
axis([0 90 -60 60])
set(gca,'Xtick',[0 15 30 45 60 75 90])
set(gca,'Ytick',[-60 0 60])
subplot(312)
plot(tout/60,sig3(14,:),'--r',tout/60,(l_f(2,:)-S(3,1))*1e6,'b',tout/60,-sig3(14,:),'--r');
set(gca,'Fontsize',12);
ylabel('kL2 (mrad)')
grid
axis([0 90 -60 60])
set(gca,'Xtick',[0 15 30 45 60 75 90])
set(gca,'Ytick',[-60 0 60])
subplot(313)
plot(tout/60,sig3(15,:),'--r',tout/60,(l_f(3,:)-S(3,2))*1e6,'b',tout/60,-sig3(15,:),'--r');
set(gca,'Fontsize',12);
ylabel('kL3 (mrad)')
grid
axis([0 90 -60 60])
set(gca,'Xtick',[0 15 30 45 60 75 90])
set(gca,'Ytick',[-60 0 60])
xlabel('Time (Min)')

