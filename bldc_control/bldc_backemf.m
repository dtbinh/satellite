function out = bldc_backemf(in)

w_e     = in(1);
theta_e = in(2); 


global CONST
ke  = CONST.ke;

phi_a = bldc_trap(theta_e);
phi_b = bldc_trap(theta_e-2*pi/3);
phi_c = bldc_trap(theta_e+2*pi/3);

E_a = ke*phi_a*w_e;     % [V]
E_b = ke*phi_b*w_e;     % [V]
E_c = ke*phi_c*w_e;     % [V]


out(1,1) = phi_a;
out(2,1) = phi_b;
out(3,1) = phi_c;
out(4,1) = E_a;
out(5,1) = E_b;
out(6,1) = E_c;
end