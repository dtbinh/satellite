function [tau_m,j]=bdotController(P,w_B_OB,B_O,R_B_O,Bdot_O)
K_d = P.K_d;
B_B = R_B_O*B_O;

S_w = [    0       -w_B_OB(3)   w_B_OB(2);
        w_B_OB(3)     0        -w_B_OB(1);
       -w_B_OB(2)   w_B_OB(1)     0 ];
 
Bdot_B  = R_B_O*Bdot_O-S_w*R_B_O*B_O;

m_B     = (-K_d/norm(B_B,2)^2)*Bdot_B;


[m_B,j] = currentScaling(P,m_B);


tau_m   = cross(m_B,B_B);
end