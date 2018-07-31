function out = bldc_discrete(Vin,theta_m,w_m)
global CONST

R  = CONST.R;   % [Ohm] Terminal Resistance               
L  = CONST.L;   % [H] Terminal Inductance, phase to phase 
kt = CONST.kt;  % [Nm/A] Torque Constant                 
ke = CONST.ke;  % [V/(rad/s)] Back-EMF Constant 
J  = CONST.J;   % [kgm^2] Rotor Inertia                
Co = CONST.Co;  % [Nm] Friction Torque Static
Cv = CONST.Cv;	% [Nm/(rad/s)] Friction Torque Dynamic 
dt = CONST.dt;


% Static Friction
if w_m < 0
    Co = -Co;
else if w_m == 0
        Co = 0;
     else
     end
end
K = -2*kt*1.17*ke/J/R - Cv/J;
M = 2*kt/J/R;
C = -Co/J;

theta_m = theta_m + (dt+0.5*K*dt^2)*w_m + 0.5*dt^2*M*Vin; 
w_m     =   w_m   + (K*dt+0.5*K^2*dt^2)*w_m + (M*dt + 0.5*dt^2*K*M)*Vin + C*dt; 

out = [theta_m;w_m];

end 