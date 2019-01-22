function out = bldc_controller(in)
w_err = in;

Kp = 41.8;
Ki = 345.32;

if u > 0
    y = 0;
else
    y = Ki*w_err;
end
sum = yint + Kp*w_err;
y = sat(sum,15);

out = Vd;
end