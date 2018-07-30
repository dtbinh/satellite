close all
clear all
clc

I_rw = 1.5465e-6;

% GAIN FOR RW DYNAMICS
k = 1.000000000000000;
d = 2.784230231948520;
t = 0.556846046389705;


%% TIME 
dt     = 0.01;        % [sec] time step
tdur   = 10;          % [sec] time at final
tspan  = 0:dt:tdur;   % [sec] time array
tlgth  = length(tspan);

%% SIMULATION
T(1) = 0;
w(1) = 0;
w_tgt(1) = 1000*2*pi/60;
h(1)  = 0;

for i = 1:1:tlgth-1
    
    Tdot(i) = I_rw*(-2*d/I_rw*T(i) - 1/t^2*w(i) + k/t^2*w_tgt(i));
    
    T(i+1)  = T(i) + Tdot(i)*dt;
    h(i+1)  = h(i) + T(i)*dt;
    w(i+1)  = 1/I_rw*h(i);
    w_tgt(i+1) = w_tgt(i);
    
    
end

figure
subplot(3,1,1)
plot(tspan,w)
grid on; hold on;
plot(tspan,w_tgt);
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
