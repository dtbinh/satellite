function out = bldc_trap2(in)
if in < 0
    theta_e = in;


if theta_e >= 0 && theta_e < 2*pi/3
    out = 1;
end

if theta_e >= 2*pi/3 && theta_e < pi
    out = 1 - 6/pi*(theta-2*pi/3);
end

if theta_e >= pi && theta_e < 5*pi/3
    out = 1;
end

if theta_e >= 5*pi/3 && theta_e < 2*pi
    out = 1 + 6/pi*(theta-2*pi/3);
end

end