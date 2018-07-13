function [tau_m,j]=referenceController(P,w_B_OB,B_B,q)
K_d = P.K_d;
K_p = P.K_p;
eps = q(2:4);

m_B     = -(K_d/norm(B_B,2)^2)*cross(B_B,w_B_OB)-(K_p/norm(B_B,2)^2)*cross(B_B,eps);


[m_B,j] = currentScaling(P,m_B); % Scaling magnetic moment to prevent exceeding of Maximum Voltage
tau_m   = cross(m_B,B_B);

end