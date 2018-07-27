function out = bldc_backemf(in)

w_m     = in(1);
theta_e = in(2); 
i_a     = in(3);
i_b     = in(4);
i_c     = in(5);

global CONST

kt = CONST.kt;
ke = CONST.ke;

f_a = bldc_trap(theta_e);
f_b = bldc_trap(theta_e-2*pi/3);
f_c = bldc_trap(theta_e+2*pi/3);

Ta = kt*i_a*f_a;
Tb = kt*i_b*f_b;
Tc = kt*i_c*f_c;

Te = Ta+Tb+Tc;

E_a = ke*f_a*w_m;     % [V]
E_b = ke*f_b*w_m;     % [V]
E_c = ke*f_c*w_m;     % [V]

out(1,1) = E_a;
out(2,1) = E_b;
out(3,1) = E_c;
out(4,1) = Te;
end