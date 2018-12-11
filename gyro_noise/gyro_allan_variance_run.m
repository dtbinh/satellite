close all
clear all
clc
fprintf('---------ALLAN VARIANCE ANALYSIS SIMULATION--------');
fprintf('\nInitialising Parameters\n');

%% LOAD DATA
file   = 'trisat_gyro2_long.dat';
fileID = fopen(file);
wdata  = textscan(fileID,'%19c %f %f %f | %4c %f %11c %f %6c');
fclose(fileID);

% INPUT PARAMETER
w0     = wdata{2};
w1     = wdata{3};
w2     = wdata{4};
w_abs  = wdata{6};
tout   = wdata{8};
tout   = (tout-tout(1))/1000;
    
% STANDARD DEVATION
fprintf('Mean   :%8.5f | %8.5f | %8.5f [dps]\n',mean(w0),mean(w1),mean(w2));
fprintf('1-Std  :%8.5f | %8.5f | %8.5f [dps]\n',std(w0),std(w1),std(w2));
fprintf('3-Std  :%8.5f | %8.5f | %8.5f [dps]\n',std(w0)*3,std(w1)*3,std(w2)*3);
    
y = w2;

N  = size(tout(:,1),1);
dt = 0.2;
tdur = tout(end);
%% ALLAN VARIANCE
tau_array = [0.2 0.4 0.6 0.8 1.0 1.2 1.4 1.6 1.8 2.0 2.2 2.4 2.6 2.8 3.0 4.0 5.0 6.0 7.0 8.0 9.0 10.0...
    12.0 14.0 16.0 18.0 20.0 22 24 26 28 30 34 38 42 46 50 60 70 80 90 100 120 140 160 180 200 240 280 320 ...
    360 400 500 600 700 800 900 1000 1200 1400 1600 2000 2400 2600 2800 3000 3200 3400 3600 3800 4000 ...
    5000 6000];

% for each tau value
for i = 1:1:length(tau_array)
fprintf('\n\n---------Running Allan Variance Calculation--------\n');
    
tau = tau_array(i);    
M   = tau/dt; % [] Number of Points per Cluster
K   = N/M;    % [] Number of Clusters

fprintf('Number of Samples  = %.2f [-]\n',N);
fprintf('Test Duration      = %.2f [s]\n',tdur);
fprintf('Sampling Period    = %.2f [s]\n',dt);
fprintf('Cluster Period     = %.2f [s]\n',tau);
fprintf('Cluster Samples    = %d \n',M);
fprintf('Number of Clusters = %d \n',K);

% index for timeset
k = 1; 

% for each measurement
for j = 1:M:N-M+2

% Initialise for Time Set
omega(k)= 0;

% Summing up omega for n samples at m set
j = uint32(j);
for m = j:1:j+M-2
    omega(k) = omega(k) + y(m);
end

% Average value of each cluster
omega(k) = omega(k)/M;       % average of omega for each time set

% Next Time Step
k  = k+1;

end

sum = 0;
for k = 1:K-1
    sum = sum + (omega(k+1)-omega(k))^2;
end
var_A(i) = 1/2/(k-1)*sum*(180/pi)^2;
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
fprintf('\nEstimated ARW = %.6f [deg/s^0.5]',sig_ARW);
fprintf('\nEstimated RRW = %.6f [deg/s^1.5]', 2.0e-4);    
fprintf('\n');
%% ALLAN VARIANCE PLOT
xmin = 1e-1;
xmax = 1e4;
ymin = 1e-2;
ymax = 1e0;

x = 0.001: 0.1: 10000;
y = 0.29*x.^(-1/2);
z = 0.0014*x.^(1/2);

figure
loglog(tau_array,sqrt(var_A),'x-')
grid on;hold on;
xlabel('tau [s]');
ylabel('Allan deviation [dps]');
loglog(x,y,'k');
loglog(x,z,'k');
loglog([1 1], [1e-5 sig_ARW], 'r')
loglog([1e-5 1],[sig_ARW sig_ARW], 'r')
axis([xmin xmax ymin ymax]);


