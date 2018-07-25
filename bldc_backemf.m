function out = bldc_backemf(in)

w_e     = in(1);
theta_e = in(2); 



ke  = 9.4e-5/(2*pi/60);
phi_m = 0.2158;

phi_a = phi_m*bldc_trap(theta_e);
phi_b = phi_m*bldc_trap(theta_e-2*pi/3);
phi_c = phi_m*bldc_trap(theta_e-4*pi/3);

E_a = phi_a*w_e;            % [V]
E_b = phi_b*w_e;     % [V]
E_c = phi_c*w_e;     % [V]


out(1,1) = phi_a;
out(2,1) = phi_b;
out(3,1) = phi_c;
out(4,1) = E_a;
out(5,1) = E_b;
out(6,1) = E_c;
end