function [B_O,Bdot_O] = dipoleField(P,lat)
w_O = P.w_O; % [rad/s] Orbit Velocity
B0  = P.B0;  % [T]     Magnetic Field at Earth Equator

B_O    = [ B0*cos(lat); 
               0      ; 
         2*B0*sin(lat)];
     
Bdot_O = [-w_O*B0*sin(lat);0; 2*w_O*B0*cos(lat)];

end