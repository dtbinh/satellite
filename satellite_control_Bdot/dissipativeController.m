function [tau_m,j]=dissipativeController(P,w_B_OB,B_B)
K_d = P.K_d;

m_B     = -(K_d/norm(B_B,2)^2)*cross(B_B, w_B_OB);


[m_B,j] = currentScaling(P,m_B);


tau_m   = cross(m_B,B_B);
end