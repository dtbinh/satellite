function Wdot = torque2omegadot(input)
% This function takes input of applied torque (T) in component
% elements, current angular velocity (W) in component
% elements, and the moment-of-inertia matrix (J) as a diagonal
% matrix containing the MOIs for the principal axes of the body
% along the diagonal. Angular acceleration is then computed and
% output as a 3x1 vector (Wdot).
T = input(1:3,1);
W = input(4:6,1);

global CONST
J = CONST.J;

Wx = W(1); 
Wy = W(2); 
Wz = W(3);

Jxx = J(1,1); 
Jyy = J(2,2); 
Jzz = J(3,3);

Tx = T(1); 
Ty = T(2); 
Tz = T(3);

wdotx = (Tx-(Jzz-Jyy)*Wz*Wy)/Jxx;
wdoty = (Ty-(Jxx-Jzz)*Wx*Wz)/Jyy;
wdotz = (Tz-(Jyy-Jxx)*Wy*Wx)/Jzz;


Wdot = [wdotx; wdoty; wdotz];