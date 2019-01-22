function out = bldc_backemf(in)

w_e     = in(1);
theta_e = in(2); 

global CONST

% Trapezium Signal based on Rotation Angle
f_a = bldc_trap(theta_e);
f_b = bldc_trap(theta_e-2*pi/3);
f_c = bldc_trap(theta_e+2*pi/3);

% Back EMF
E_a = CONST.ke*f_a*w_e;     % [V]
E_b = CONST.ke*f_b*w_e;     % [V]
E_c = CONST.ke*f_c*w_e;     % [V]


out(1,1) = f_a;
out(2,1) = f_b;
out(3,1) = f_c;
out(4,1) = E_a;
out(5,1) = E_b;
out(6,1) = E_c;

end