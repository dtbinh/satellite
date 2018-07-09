function [B_O,Bdot_O] = magneticFieldDipole(P,lat)
global CONST

w_O = CONST.w_O; % [rad/s] Orbit Velocity
B0  = CONST.B0;  % [T]     Magnetic Field at Earth Equator

B_O    = [ B0*cos(lat) ; 
               0       ; 
         2*B0*sin(lat)];
     
Bdot_O = [-w_O*B0*sin(lat);
                  0       ; 
         2*w_O*B0*cos(lat)];

end