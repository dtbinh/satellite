function out = bldc_trap(in)

out = cos(in);
out = 1/0.5*sat(out,0.5);

end