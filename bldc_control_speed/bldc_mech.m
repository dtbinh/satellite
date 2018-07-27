function out = bldc_mech(in)

Te   = in(1);
To    = in(2);
w_m   = in(3);

global CONST

kt = CONST.kt;     
J  = CONST.J; 
Cv = CONST.Cv;
Co = CONST.Co;

% Mechanical Dynamics
wdot_m = 1/J*(Te-To-Cv*w_m-Co);

out(1,1) = wdot_m;

end