function [r, v,Q_pX] = sv_from_coe(mu,coe) 
% computes the state vector (r,v) from the classical orbital elements (coe) 
% ------------------------------------------------------------ 
% This function computes the state vector (r,v) from the 
% classical orbital elements (coe). 
% 
% mu    - gravitational parameter (km^3; s^2) 
% coe   - orbital elements [h e RA incl w TA] 
%         where 
%           h   = angular momentum (km^2/s) 
%           e   = eccentricity 
%           RA  = right ascension of the ascending node (rad) 
%           incl= inclination of the orbit (rad) 
%           w   = argument of perigee (rad) 
%           TA  = true anomaly (rad) 
% R3_w  - Rotation matrix about the z-axis through the angle w 
% R1_i  - Rotation matrix about the x-axis through the angle i 
% R3_W  - Rotation matrix about the z-axis through the angle RA 
% Q_pX  - Matrix of the transformation from perifocal to 
%         geocentric equatorial frame 
% rp    - position vector in the perifocal frame (km) 
% vp    - velocity vector in the perifocal frame (km/s) 
% r     - position vector in the geocentric equatorial frame (km) 
% v     - velocity vector in the geocentric equatorial frame (km/s) 
% 
% User M-functions required: none 
% ------------------------------------------------------------ 
%   This .m file was from Appendix D of the book: 
%       <Orbit Mechanics for engineering Students> (Howard D. Curtis) 
%   You could get appendix D from: http://books.elsevier.com/companions 
% ------------------------------------------------------------ 
%   Last Edit by:           Li yunfei   2008/07/31 
% ------------------------------------------------------------ 
 
 
h    = coe(1); % angular momentum (km^2/s) 
e    = coe(2); % eccentricity 
RA   = coe(3); % right ascension of the ascending node (rad) 
incl = coe(4); % inclination of the orbit (rad) 
w    = coe(5); % argument of perigee (rad) angle between ascending node to perigee
TA   = coe(6); % true anomaly (rad) angle between perigee to satellite

%...Equations 4.37 and 4.38 (rp and vp are column vectors) in perifocal frame (xaxis is perigee, z axis is normal to orbit plane) 
rp = (h^2/mu) * (1/(1 + e*cos(TA))) * (cos(TA)*[1;0;0] + sin(TA)*[0;1;0]);
vp = (mu/h) * (-sin(TA)*[1;0;0] + (e + cos(TA))*[0;1;0]);

%...Equation 4.39: 
R3_W = [ cos(RA) sin(RA)    0 
        -sin(RA) cos(RA)    0 
            0       0       1]; 
       
%...Equation 4.40: 
R1_i = [1       0           0 
        0 cos(incl)     sin(incl) 
        0 -sin(incl)    cos(incl)];
   
%...Equation 4.41: 
R3_w = [ cos(w) sin(w)  0 
        -sin(w) cos(w)  0 
            0       0   1]; 
         
%...Equation 4.44: 
Q_pX = R3_W'*R1_i'*R3_w'; % Perifocal Frame to Inertia Frame 

%...Equations 4.46 (r and v are column vectors): 
r = Q_pX*rp;
v = Q_pX*vp;
 