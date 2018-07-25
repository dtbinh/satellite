function out = bldc_speed(in)

phi_a = in(1);
phi_b = in(2); 
phi_c = in(3);

i_a = in(4);
i_b = in(5); 
i_c = in(6);

Ti  = in(7);
w_m = in(8);

kt = 4/2;      % [Nm/A] Torque constant
J  = 5.5e-3; 
Cv = 2e-3;

Ta = kt*i_a*phi_a;
Tb = kt*i_b*phi_b;
Tc = kt*i_c*phi_c;

Te = Ta+Tb+Tc;

wdot_m = 1/J*(Te-Ti-Cv*w_m);

out(1,1) = wdot_m;

end