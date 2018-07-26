close all
clear all
clc

dt   = 0.01;
tdur = 1;
tspan= 0:dt:tdur;
tstep= length(tspan);

theta(1) = 0;    %[rad]
omega = 100*2*pi/60; %[rad/s]
for i=2:1:tstep
    theta(i) = theta(i-1) + omega*dt;
    if abs(theta(i))>2*pi
        theta(i) = theta(i)-sign(theta(i))*2*pi;
    end
    out1(i) = bldc_trap(theta(i));
    out2(i) = bldc_trap2(theta(i));
    
    
    
    
end

%% PLOT
figure

subplot(3,1,1)
plot(tspan,theta*rad2deg,'x-')
grid on;
xlabel('Time [s]');
ylabel('Theta [deg]');

subplot(3,1,2)
plot(tspan,out1,'x-')
grid on;
xlabel('Time [s]');
ylabel('Output');

subplot(3,1,3)
plot(tspan,out2,'x-')
grid on;
xlabel('Time [s]');
ylabel('Output');

