function [m_B,j] = currentScaling(P,m_B)
Nx = P.Nx;
Ny = P.Ny;
Nz = P.Nz;

Ax = P.Ax;
Ay = P.Ay;
Az = P.Az;

ix_max = P.ix_max;
iy_max = P.iy_max;
iz_max = P.iz_max;

V = P.V;

% Desired current
ix = m_B(1)/(Nx*Ax);
iy = m_B(2)/(Ny*Ay);
iz = m_B(3)/(Nz*Az);

ratio(1) = ix_max/abs(ix);
ratio(2) = iy_max/abs(iy);
ratio(3) = iz_max/abs(iz);

% IF desired current is more than maximum allowed current
if min(ratio)<1
    m_B = m_B*min(ratio);
end


Wx = abs(V*m_B(1)/(Nx*Ax)); % [W] Power Consumption
Wy = abs(V*m_B(2)/(Ny*Ay)); % [W] Power Consumption
Wz = abs(V*m_B(3)/(Nz*Az)); % [W] Power Consumption

j  = Wx+Wy+Wz;              % [W] Total Power Consumption

% Variable Storage
global tmpVAR;
tmpVAR.moment = m_B;
tmpVAR.W = Wx+Wy+Wz;

end