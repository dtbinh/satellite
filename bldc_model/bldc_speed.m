function out = bldc_speed(in)

f_a = in(1);
f_b = in(2); 
f_c = in(3);

i_a   = in(4);
i_b   = in(5); 
i_c   = in(6);

To    = in(7);
w_m   = in(8);

global CONST

Cv = CONST.Cv;

% Electrical Torque
Ta = CONST.kt*i_a*f_a;
Tb = CONST.kt*i_b*f_b;
Tc = CONST.kt*i_c*f_c; 

Te = Ta+Tb+Tc;

% Static Friction
if w_m == 0;
    if Te < 0
        Co = -CONST.Co;
    else 
        Co = CONST.Co;
    end
else
    Co = 0;
end

frict = -Cv*w_m-Co;    

% Mechanical Dynamics
wdot_m = 1/CONST.J*(Te+frict+To);

out(1,1) = wdot_m;   
out(2,1) = Te;       % Electrical Torque

end