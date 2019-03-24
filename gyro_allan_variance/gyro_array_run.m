% -------------------------------------------------------------------------
% GYRO ARRAY
% -------------------------------------------------------------------------
% This program runs the gyro array filter using first-order Markov Process
% to reduce the ARW of the gyro array output
% ------------------------------------------------------------------------
close all
clear all
clc
fprintf('---------GYRO ARRAY SIMULATION--------');
fprintf('\nInitialising Parameters');
%% SIMULATION PARAMETER
dt   = 1/200;           % [s] Simulation Step
tdur = 3600;            % [s] Simulation Run Time
Nt    = tdur/dt;        % []  Simulation Step Number
tout = 0:dt:tdur-dt;    % []  Simulation Array
%% MEMS GYRO PARAMETER
BW       = 40;           % [Hz] System’s band width
tau      = 1/BW;         % [s] 
sig_v    = 6.17/60/180*pi;        % [rad/s^0.5] Angular Random Walk
sig_u    = 294.28/3600/60/180*pi;    % [rad/s^1.5] Rate Random Walk
bias(:,1)  = [0.0; 0.0; 0.0;0.0;0.0;0.0];
w(1)     = 0;
wtrue(1) = 0;
N        = 6;             % Number of Gyro in Array
is_bias_on = 1;
%% DISCRETE GYRO MODELING MEASUREMENT 
fprintf('\nGenerating Gyro Modelling Measurement\n');
for i = 1:1:Nt-1
   wtrue(i+1) = 0.0;%0.01*sin(dt*i*rand(1,1));
   
   for j = 1:N
    w(j,i+1)    = wtrue(i+1) + sqrt(sig_v^2/tau)*randn(1,1);
    if (is_bias_on)
        bias(j,i+1) = bias(j,i) + sig_u*tau^0.5*randn(1,1);
        w(j,i+1) = w(j,i+1)+0.5*(bias(j,i+1)+bias(j,i));
    end
   end
end


%% ESTIMATING
Xk       = 0;
qw       = 0.0772; %[deg^2/s^3]
tw       = 500;    %[s] Process Time Constant

H  = ones(N,1);
R  = sig_v^2*eye(N);
for i = 1:1:Nt
% Measurement
Zk = w(:,i);


D  = H'*R^-1*H;
Kinf = D^-1*(-1/tw + sqrt((1/tw)^2 + D*qw) ) * H' * R^-1;
A = -(1/tw + Kinf*H);    

Xk    = exp(A*dt)*Xk + A^-1*(exp(A*dt)-1)*Kinf*Zk;

w_est(i) = Xk;
end

%% PLOT

figure
subplot(2,1,1)
for j=1:N
    plot(tout,w(j,:),':'); hold on;
end
grid on;
plot(tout,w_est,'k');
% axis([0 inf 0.28 0.32]);

subplot(2,1,2)
plot(tout,w_est-wtrue,'r');
grid on;

fprintf('Std Dev:\n');
for j=1:N
    fprintf(' gyro[%2d]: %9.6f [deg/s] \n',j,std(w(j,:)-wtrue)/pi*180);
end
fprintf('\n');
fprintf('Single Gyroscope :\n');
fprintf('   ARW: %.6f [deg/s^0.5]\n',sig_v*180/pi);
fprintf('   ARW: %.6f [deg/h^0.5]\n',sig_v*180/pi*60);
fprintf('Virtual Gyroscope:\n');
fprintf('   STD: %.6f [deg/s]\n',std(w_est-wtrue)/pi*180);
fprintf('   ARW: %.6f [deg/s^0.5]\n',std(w_est-wtrue)/pi*180/sqrt(BW));
fprintf('   ARW: %.6f [deg/h^0.5]\n',std(w_est-wtrue)/pi*180/sqrt(BW)*60);


%% ALLAN VARIANCE
% input
y = w_est;
tau_array = [0.01 0.05 0.1 0.2 0.5 1 2 5 10 20 50 100 200 300 400 500 600 700 800 900 1000 1100 ...
    1200 1400 1600 1800];

% for each tau value
for t = 1:1:length(tau_array)
fprintf('\n\n---------Running Allan Variance Calculation--------\n');
    
tau = tau_array(t);    
M   = tau/dt; % [] Number of Points per Cluster
K   = Nt/M;    % [] Number of Clusters

fprintf('Number of Samples  = %.3f [-]\n',Nt);
fprintf('Test Duration      = %.3f [s]\n',tdur);
fprintf('Sampling Period    = %.3f [s]\n',dt);
fprintf('Cluster Period     = %.3f [s]\n',tau);
fprintf('Cluster Samples    = %d \n',M);
fprintf('Number of Clusters = %d \n',K);

k = 1; % index for timeset
% for each measurement
for j = 1:M:Nt-M+2

    % Initialise for Time Set
    omega(k)= 0;
    
    % Summong up omega for n samples at m set
   
    for m = j:1:j+M-1
        omega(k) = omega(k) + y(m);
    end

    % Average value of each cluster
    omega(k) = omega(k)/M;       % average of omega for each time set
%     mean(k)  = mean2(y(j:j+M)); % check for omega(k)

    % Next Time Step
    k  = k+1;
end

sum = 0;
for k = 1:K-1
    sum = sum + (omega(k+1)-omega(k))^2;
end
var_A(t) = 1/2/(k-1)*sum*(180/pi)^2;
end

%% PLOT
figure
plot(tout,y(1:end))
grid on;
text(0,0,strcat(' STD:',num2str(std(y))));
axis([-inf inf -inf inf])

%% ALLAN VARIANCE REPORT

[row,col] = find(tau_array==1);
sig_ARW = sqrt(var_A(col));

fprintf('\n\n---------------ALLAN VARIANCE REPORT--------------')
fprintf('\nGyro Model Hz = %d   [Hz]',1/dt);
fprintf('\nStandard Dev  = %.6f [deg/s]',std(y));
fprintf('\nActual ARW    = %.6f [deg/s^0.5]',sig_v/pi*180);
fprintf('\nEstimated ARW = %.6f [deg/s^0.5]',sig_ARW);
fprintf('\nActual RRW    = %.6f [deg/s^1.5]',sig_u/pi*180);
fprintf('\nEstimated ARW = %.6f [deg/s^1.5]', 2.0e-4);    
fprintf('\n');
%% ALLAN VARIANCE PLOT
close all
xmin = 1e-2;
xmax = 1e4;
ymin = 1e-4;
ymax = 1e1;

x = 0.001: 0.1: 10000;
y = sig_ARW*x.^(-1/2);
z = 0.0001155*x.^(1/2);
biasline = 0.0025;

figure
loglog(tau_array,sqrt(var_A),'x-')
grid on;hold on;
xlabel('tau [s]');
ylabel('Allan deviation [dps]');
loglog(x,y,'k');
loglog(x,z,'k');
loglog([1e-5 1e4],[biasline biasline],  'g')
loglog([1 1], [1e-5 sig_ARW], 'r')
loglog([1e-5 1],[sig_ARW sig_ARW], 'r')
axis([xmin xmax ymin ymax]);