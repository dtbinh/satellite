function out = bldc_motor(in)
global CONST

R  = CONST.R;   % [Ohm] Terminal Resistance               
L  = CONST.L;   % [H] Terminal Inductance, phase to phase 
kt = CONST.kt;  % [Nm/A] Torque Constant                 
ke = CONST.ke;  % [V/(rad/s)] Back-EMF Constant 
J  = CONST.J;   % [kgm^2] Rotor Inertia                
Co = CONST.Co;  % [Nm] Friction Torque Static
Cv = CONST.Cv;	% [Nm/(rad/s)] Friction Torque Dynamic 

V   = in(1);
w_m = in(2);
To  = in(3);

% Static Friction
if w_m < 0
    Co = -Co;
else if w_m == 0
        Co = 0;
     else
     end
end

i      = 1/R*(V-1.17*ke*w_m);
wdot_m = 1/J*(2*kt*i-Cv*w_m-Co+To);

out = [wdot_m;i];
end