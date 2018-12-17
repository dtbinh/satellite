% -------------------------------------------------------------------------
% UKF PENDULUM
% -------------------------------------------------------------------------
% This program runs the estimation using Unscented Kalman Filter for a
% simple pendulum model
% 
% Author: Rusty Goh
% Date:   17 Dec 2018 
% 
% ------------------------------------------------------------------------

close all
clear all
clc
global CONST

dt = 0.05;
tdur = 100;
sig_noise = 1e-2;
CONST.model = 'ukf_pendulum_model';
CONST.R = sig_noise^2;
CONST.Q = diag([0.00001,0.00001,0.001]);
CONST.kappa = 1;
CONST.dt = dt;

x_0 = [deg2rad(15);0;5];
u_0 = [0;0;0];

sim(CONST.model,tdur);

%Plot
x_true=zeros;
[m,n,p]=size(x);
x_true(1:m,1:p) = x(1:m,1,1:p);

for i=1:length(tout)
    
    Pxx_est_ukf(i,1) = Pxx_est(1,1,i);
    Pxx_est_ukf(i,2) = Pxx_est(2,2,i);
    Pxx_est_ukf(i,3) = Pxx_est(3,3,i);
end

% State 1: Theta
figure

plot(tout, (x_est_ukf(1,:)-x_true(1,:))*180/pi);
grid on
title('UKF Angle Error (Deg)')
xlabel('Time (S)')
ylabel('Angle Error (Deg)')
ylim([-5 5])
hold on
plot(tout, 3*sqrt(Pxx_est_ukf(:,1))*180/pi,'-.');
plot(tout, -3*sqrt(Pxx_est_ukf(:,1))*180/pi,'-.');

figure
plot(tout, (x_true(1,:))*180/pi);
grid on; hold on
plot(tout, (x_est_ukf(1,:))*180/pi);
title('UKF Angle (Deg)')
xlabel('Time (S)')
ylabel('Angle Error (Deg)')




% State 2: Omega - Angular Rate
figure
plot(tout, (x_est_ukf(2,:)-x_true(2,:))*180/pi);
grid on
title('UKF Angular Rate Error (Deg/Sec)')
xlabel('Time (S)')
ylabel('Angular Rate Error (Deg/Sec)')
ylim([-10 10])
hold on
plot(tout, 3*sqrt(Pxx_est_ukf(:,2))*180/pi','-.');
plot(tout, -3*sqrt(Pxx_est_ukf(:,2))*180/pi','-.');


% State 3: Iy - Moment of Inertia
figure
plot(tout, x_est_ukf(3,:)-x_true(3,:));
grid on
title('UKF Moment of Inertia Error')
xlabel('Time (S)')
ylabel('Moment of Inertia (kg m\^2)')
ylim([-5 5])
hold on
plot(tout, 3*sqrt(Pxx_est_ukf(:,3))','-.');
plot(tout, -3*sqrt(Pxx_est_ukf(:,3))','-.');
