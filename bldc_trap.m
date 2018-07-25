function out = bldc_trap(in,trap,phi_m)

out = cos(in);
out = phi_m/trap*sat(out,trap);

end