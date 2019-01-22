% Date:   17 Dec 2018 
% 
% ------------------------------------------------------------------------
close all
clear all
clc
dt   = 0.1;          % [s] Gyro Sampling Time 
tdur = 60;           % [s] Time Duration
N    = tdur/dt;        % [] Number of Samples
tout = 0:dt:tdur-dt;

sig_v = 0.033/180*pi;  % [rad/s^0.5] Angular Random Walk
sig_u = 0/180*pi;   % [rad/s^1.5] Rate Random Walk

bias  = [0.00;0.0;0.0];


% Initialisation
i = 1;
bias(i+1) = bias(i)+sig_u*dt^0.5*randn(1,1);

y(i) = sqrt(sig_v^2/dt)*randn(1,1)+0.5*(bias(i+1)+bias(i));
y_filter(i) = 0;
y_avg_filter(i) = 0;
ydot_filter(i) = 0;
ydot(i) = 0;
% Average Moving Filter
cnt = 0;
y_total = 0;

% Time
t_next = 0;
t_interval = 0.4;

% DISCRETE GYRO MODELING MEASUREMENT
for i = 2:N+1

   bias(i+1) = bias(i)+sig_u*dt^0.5*randn(1,1);
   y_clean(i) = 0.01*(0.5*tout(i-1));
   y(i) = y_clean(i)+sqrt(sig_v^2/dt)*randn(1,1)+0.5*(bias(i+1)+bias(i));
   
   % Complimentary Filter
   y_filter(i) = 0.05*y(i) + 0.95*y_filter(i-1);
   
   % Average Moving Filter
   if i > 10
    % Calculate last 10 measurement
    for cnt = i:-1:i-9
       y_total = y_total + y(cnt);
    end
    
    y_avg_filter(i) = y_total/10;
    y_total = 0;
   end
   
   if tout(i-1)>t_next
       t_next = t_next+t_interval;
       ydot(i) = (y(i)-y(i-1))/t_interval;
       ydot_filter(i) = (y_filter(i)-y_filter(i-1))/t_interval;
       if i > 10
       ydot_avg_filter(i) = (y_avg_filter(i)-y_avg_filter(i-1))/t_interval;
       end
   else
         ydot(i) = ydot(i-1);
         ydot_filter(i) =  ydot_filter(i-1);
       if i > 10
        ydot_avg_filter(i) = ydot_avg_filter(i-1);
       end

   end
   
end

%% PLOT
figure
subplot(2,1,1)
plot(tout,y(1:end-1))
grid on;hold on;
plot(tout,y_filter(1:end-1));
plot(tout,y_avg_filter(1:end-1))
plot(tout,y_clean(1:end-1))
axis([-inf inf -inf inf])

subplot(2,1,2)

plot(tout,ydot(1:end-1),'.')
grid on;hold on;
plot(tout,ydot_filter(1:end-1));
plot(tout,ydot_avg_filter(1:end-1))
axis([-inf inf -0.01 0.01])

figure
plot(tout,y(1:end-1)-y_clean(1:end-1))
grid on;hold on;
plot(tout,(y_filter(1:end-1)-y_clean(1:end-1)));
plot(tout,(y_avg_filter(1:end-1)-y_clean(1:end-1)))
axis([-inf inf -inf inf])

fprintf('y std: %.6f \n',std(y-y_clean));
fprintf('y_avg_filter std: %.6f \n',std(y_avg_filter-y_clean));
fprintf('y_filter std: %.6f \n',std(y_filter-y_clean));