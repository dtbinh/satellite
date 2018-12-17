% -------------------------------------------------------------------------
% GYRO ALLAN VARIANCE PLOT
% -------------------------------------------------------------------------
% This program plots the gyro allan variance from .dat files. Data
% measurements are to be at least 3 hours for good plots. The format of the
% file is as follow:
% 
% Gyro values [b-fr]:  -0.006  -0.001   0.024 | Abs.   0.025 [dps] @ t=        1307811[msec]
% Gyro values [b-fr]:   0.018   0.004   0.042 | Abs.   0.046 [dps] @ t=        1308912[msec]
% Gyro values [b-fr]:   0.048   0.010   0.011 | Abs.   0.050 [dps] @ t=        1310012[msec]
%
% Author: Rusty Goh
% Date:   17 Dec 2018 
% ------------------------------------------------------------------------

close all
clear all
clc
fprintf('---------ALLAN VARIANCE ANALYSIS SIMULATION--------');
fprintf('\nInitialising Parameters\n');
% ------------------------------------------------------------------------
% LOAD DATA
file   = 'trisat_gyro0.dat';
fileID = fopen(file);
wdata  = textscan(fileID,'%19c %f %f %f | %4c %f %11c %f %6c');
fclose(fileID);
% ------------------------------------------------------------------------
% INPUT PARAMETER
w0     = wdata{2};
w1     = wdata{3};
w2     = wdata{4};
w_abs  = wdata{6};
tout   = wdata{8};
tout   = (tout-tout(1))/1000;
% ------------------------------------------------------------------------    
% STANDARD DEVATION
fprintf('Mean   :%8.5f | %8.5f | %8.5f [dps]\n',mean(w0),mean(w1),mean(w2));
fprintf('1-Std  :%8.5f | %8.5f | %8.5f [dps]\n',std(w0),std(w1),std(w2));
fprintf('3-Std  :%8.5f | %8.5f | %8.5f [dps]\n',std(w0)*3,std(w1)*3,std(w2)*3);

% ------------------------------------------------------------------------
% PARAMETERS FOR ALLAN VARIANCE
kk = 2;

switch kk
    case 0
        y  = w0;                
    case 1
        y  = w1;                
    case 2
        y  = w2;
    otherwise
end
        
N  = size(tout(:,1),1); % [] Size of Data
for ii=1:1:length(tout)-1
    
    dtarray(ii) = tout(ii+1)-tout(ii);   % [sec] Time at which the .dat file is measured
end
dt   = mean(dtarray);
type = 2;              % type 1 for dt = 0.2, type 2 for dt = 1.0
tdur = tout(end);       % [sec] Duration of Measurement 

% ------------------------------------------------------------------------
% ALLAN VARIANCE

switch type
    case 1
    tau_array = [0.2 0.4 0.6 0.8 1.0 1.2 1.4 1.6 1.8 2.0 2.2 2.4 2.6 2.8 3.0 4.0 5.0 6.0 7.0 8.0 9.0 10.0...
        12.0 14.0 16.0 18.0 20.0 22 24 26 28 30 34 38 42 46 50 60 70 80 90 100 120 140 160 180 200 240 280 320 ...
        360 400 500 600 700 800 900 1000 1200 1400 1600 2000 2400 2600 2800 3000 3200 3400 3600 3800 4000 ...
        5000 6000];
    case 2
    tau_array = [1.0 2.0 3.0 4.0 5.0 6.0 7.0 8.0 9.0 10.0...
        12.0 14.0 16.0 18.0 20.0 22 24 26 28 30 34 38 42 46 50 60 70 80 90 100 120 140 160 180 200 240 280];
end


maxjj = 0;

for jj = 1:1:length(tau_array)

    if tau_array(jj)<dt
        minjj = jj+1;
    end
    if tau_array(jj)<0.2*tdur
        maxjj = jj;
    end
end

% ------------------------------------------------------------------------
% for each tau value
for i = 1:1:maxjj
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

if sum == 0
    var_A(i) = 0;
else
    var_A(i) = 1/2/(k-1)*sum*(180/pi)^2;
end

end
% ------------------------------------------------------------------------
screensize   = get(0,'ScreenSize');
screenwidth  = screensize(3);
screenheight = screensize(4);
screennumber = 0;

% ------------------------------------------------------------------------
% PLOT

fig = figure;
set(fig,'Position',[screenwidth*(screennumber+0) 0 screenwidth*0.4 screenheight]);

subplot(2,1,1)
plot(tout,y)
grid on;
text(0,0,strcat(' STD:',num2str(std(y))));
axis([-inf inf -inf inf])
xlabel('Time [s]');
ylabel('Gyro Measurement [dps]');


% ------------------------------------------------------------------------
% ALLAN VARIANCE REPORT

[row,col] = find(tau_array==1);

switch kk
    case 0
        sig_ARW = 7.00;                  % [deg/s^0.5] Adjust this value with plot
        sig_RRW = 0.001;                 % [deg/s^1.5] Adjust this value with plot
        BW      =  1/((std(y)/sig_ARW)^2); % Bandwidth                
    case 1
        sig_ARW = 6.00;                  % [deg/s^0.5] Adjust this value with plot
        sig_RRW = 0.0015;                 % [deg/s^1.5] Adjust this value with plot
        BW      =  1/((std(y)/sig_ARW)^2); % Bandwidth                
    case 2
        sig_ARW = 8.00;                    % [deg/s^0.5] Adjust this value with plot
        sig_RRW = 0.002;                   % [deg/s^1.5] Adjust this value with plot
        BW      =  1/((std(y)/sig_ARW)^2); % Bandwidth
    otherwise
end

x = 0.001: 0.1: 10000;
yline = sig_ARW*x.^(-1/2);
zline = (sig_RRW/1.75)*x.^(1/2);

% ------------------------------------------------------------------------
% ALLAN VARIANCE PLOT
xmin = 1e-1;
xmax = 1e4;
ymin = 1e-3;
ymax = 1e0;

subplot(2,1,2)
loglog(tau_array(1:maxjj),sqrt(var_A),'x-')
grid on;hold on;
xlabel('tau [s]');
ylabel('Allan deviation [dps]');
loglog(x,yline,'k');
loglog(x,zline,'k');
loglog([1 1], [1e-5 sig_ARW], 'r')
loglog([1e-5 1],[sig_ARW sig_ARW], 'r')
loglog([3 3], [1e-5 sig_RRW], 'r')
loglog([1e-5 3],[sig_RRW sig_RRW], 'r')
% axis([xmin xmax ymin ymax]);

% ------------------------------------------------------------------------
% Report
fprintf('\n\n---------------ALLAN VARIANCE REPORT--------------')
fprintf('\nGyro Model Hz = %d   [Hz]',1/dt);
fprintf('\nStandard Dev  = %.6f [deg/s]',std(y));
fprintf('\nEstimated ARW = %.6f [deg/s^0.5]', sig_ARW);
fprintf('\nEstimated RRW = %.6f [deg/s^1.5]', sig_RRW);
fprintf('\nEstimated BW  = %.6f [Hz]', BW);
fprintf('\n');
fprintf('\nBack calculate Std Dev  = %.6f [dps]', sig_ARW*(1/BW)^0.5);
fprintf('\n');

filename = strcat('trisat_hpm_gyro',num2str(kk));
savepdf(fig,filename);