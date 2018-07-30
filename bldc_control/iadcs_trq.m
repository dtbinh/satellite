close all
clear all
clc

I_rw = 1.5465e-6;

% PID GAINS FOR TORQUE CONTROL
kp = 76;
ki = 477; 
kd = 0.0;

%% TIME 
dt     = 0.01;      % [sec] time step
tdur   = 10;          % [sec] time at final
tspan  = 0:dt:tdur;  % [sec] time array
tlgth  = length(tspan);

%% SIMULATION
T(1) = 0;
w(1) = 0;

h(1)  = 0;

for i = 1:1:tlgth-1
    T_tgt(i) = 0.05e-3;
    
    wdot(i) = T_tgt(i)/I_rw;
    hdot(i) = I_rw*wdot(i);
    T(i+1)  = hdot(i);
    
    h(i+1)  = h(i) + T(i)*dt;
    w(i+1)  = 1/I_rw*h(i);
    
end

figure
subplot(3,1,1)
plot(tspan,w)
grid on; hold on;
xlabel('Time [s]');
ylabel('Angular Velocity [rad/s]');

subplot(3,1,2)
plot(tspan,h);
grid on; hold on;
xlabel('Time [s]');
ylabel('Angular Momentum [kgm^2/s]');

subplot(3,1,3)
plot(tspan,T);
grid on; hold on;
xlabel('Time [s]');
ylabel('Torque [Nm]');
