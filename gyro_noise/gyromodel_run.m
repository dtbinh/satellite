close all
clear all
clc
fprintf('---------ALLAN VARIANCE ANALYSIS SIMULATION--------');
fprintf('\nInitialising Parameters');
dt   = 1/10;          % [s] Gyro Sampling Time 
tdur = 18000;          % [s] Time Duration
N    = tdur/dt;        % [] Number of Samples
tout = 0:dt:tdur;

sig_v   = 0.033/180*pi;  % [rad/s^0.5] Angular Random Walk - Noise
sig_u   = 2e-4/180*pi;   % [rad/s^1.5] Rate Random Walk - Bias Drift
bias(1) = 15.2/180*pi;   % [rad/s] Bias Base

%% DISCRETE GYRO MODELING MEASUREMENT
fprintf('\nGenerating Gyro Modelling Measurement');
for i = 1:N+1
   bias(i+1) = bias(i)+sig_u*dt^0.5*randn(1,1);
   y(i) = sqrt(sig_v^2/dt)*randn(1,1)+0.5*(bias(i+1)+bias(i));
   
    
end

%% ALLAN VARIANCE
tau_array = [0.01 0.05 0.1 0.2 0.5 1 2 5 10 20 50 100 200 300 400 500 600 700 800 900 1000 1100 ...
    1200 1400 1600 1800 2000 2200 2400 2600 2800 3000 3500 4000 4500 5000 5500 6000 6500 7000 7500 8000 ...
    8500 9000];

% for each tau value
for t = 1:1:length(tau_array)
fprintf('\n\n---------Running Allan Variance Calculation--------\n');
    
tau = tau_array(t);    
M   = tau/dt; % [] Number of Points per Cluster
K   = N/M;    % [] Number of Clusters

fprintf('Number of Samples  = %.2f [-]\n',N);
fprintf('Test Duration      = %.2f [s]\n',tdur);
fprintf('Sampling Period    = %.2f [s]\n',dt);
fprintf('Cluster Period     = %.2f [s]\n',tau);
fprintf('Cluster Samples    = %d \n',M);
fprintf('Number of Clusters = %d \n',K);

k = 1; % index for timeset
% for each measurement
for j = 1:M:N-M+2

    % Initialise for Time Set
    omega(k)= 0;
    
    % Summong up omega for n samples at m set
   j = uint32(j);
    for m = j:1:j+M-2
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
plot(tout,y)
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
fprintf('\nEstimated RRW = %.6f [deg/s^1.5]', 2.0e-4);    
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
loglog([3 3], [1e-5 sig_u/pi*180], 'r')
loglog([1e-5 3],[sig_u/pi*180 sig_u/pi*180], 'r')
axis([xmin xmax ymin ymax]);


