function f = bldc_dynamics(t,x)
ia      = x(1,1);
ib      = x(2,1);
ic      = x(3,1);
w_m     = x(4,1);
w_e     = x(5,1);
theta_m = x(6,1);
theta_e = x(7,1);

global CONST
R   = CONST.R;   % [Ohm] Terminal Resistance
L   = CONST.L;   % [H] Terminal Inductance, phase to phase
eff = CONST.eff; % [-] Efficiency
kt  = CONST.kt;  % [Nm/A] Torque Constant
ki  = CONST.ki;  % [A/Nm] Current constant
ke  = CONST.ke;  % [V/(rad/s)] Back-EMF Constant
kn  = CONST.kn;  % [(rad/s)/V] Speed Constant
J   = CONST.J;   % [kgm^2] Rotor Inertia

Co  = CONST.Co;  % [Nm] Friction Torque Static
Cv  = CONST.Cv;  % [Nm/(rad/s)] Friction Torque Dynamic


p   = CONST.p;      % [-] Number of Poles
Va = 0.0001;
Vb = 0;
Vc = 0;

% EMF
phi_a=bldc_trap(theta_e);
phi_b=bldc_trap(theta_e-2*pi/3);
phi_c=bldc_trap(theta_e+2*pi/3);

E_a = ke*phi_a*w_e;            % [V]
E_b = ke*phi_b*w_e;     % [V]
E_c = ke*phi_c*w_e;     % [V]


% Curent Calculation
idot_a = 1/L*(Va-R*ia-E_a);
idot_b = 1/L*(Vb-R*ib-E_b);
idot_c = 1/L*(Vc-R*ic-E_c);

% Electromagnetic Torque
Ta = kt*ia*phi_a;
Tb = kt*ib*phi_b;
Tc = kt*ic*phi_c;
Ti = Co;
Te = p/2*(Ta+Tb+Tc);

wdot_m = 1/J*(Te-Ti-Cv*w_m);
wdot_e = p/2*wdot_m;


f(1,1) = idot_a;
f(2,1) = idot_b;
f(3,1) = idot_c;
f(4,1) = wdot_m;
f(5,1) = wdot_e;
f(6,1) = w_m;
f(7,1) = w_e;

end