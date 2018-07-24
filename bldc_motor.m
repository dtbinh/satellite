close all
clear all
clc

% Parameters
R   = 16;       % [Ohm] Terminal Resistance
L   = 26e-6;    % [H] Terminal Inductance, phase to phase
eff = 0.51;     % [-] Efficiency
km  = 0.902e-3; % [Nm/A] Torque Constant
ki  = 1.109e3;  % [A/Nm] Current constant
ke  = 9.4e-1/(2*pi/60) ;  % [V/(rad/s)] Back-EMF Constant
kn  = 10587;    % [rpm/V] Speed Constant
J   = 0.125;    % [gcm^2] Rotor Inertia

dt    = 0.01;   % [sec]
tdur  = 600;    % [sec]
tout  = 0:dt:tdur;
w     = 1*2*pi/60; % [rad/s]
theta = 0;

for i = 1:1:tdur/dt+1
    theta  = theta + w*dt;
    
    if theta >= 2*pi
        theta = 0;

    end
    
% Back-EMF of Phase
Ea(i) = ke*bldc_trap(theta)*w;
Eb(i) = ke*bldc_trap(theta-2*pi/3)*w;
Ec(i) = ke*bldc_trap(theta+2*pi/3)*w;

end

figure
plot(tout,Ea);
grid on;
return


Va = R*ia+L*ia_dot+Ea;
Vb = R*ib+L*ib_dot+Eb;
Vc = R*ic+L*ic_dot+Ec;


