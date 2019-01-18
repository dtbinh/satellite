function out = bldc_inverter(in)

pulse1 = in(1);
pulse2 = in(2);
pulse3 = in(3); 

% Voltage across the phase
V_dc   = in(4); 

V_a = 1/3*(2*pulse1-pulse2-pulse3)*V_dc;
V_b = 1/3*(2*pulse2-pulse1-pulse3)*V_dc;
V_c = 1/3*(2*pulse3-pulse1-pulse2)*V_dc;

out(1,1) = V_a;
out(2,1) = V_b;
out(3,1) = V_c;
end