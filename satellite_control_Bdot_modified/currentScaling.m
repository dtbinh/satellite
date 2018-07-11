function [m_B,j] = currentScaling(m_B)
global CONST

Nx = CONST.Ncoil(1);
Ny = CONST.Ncoil(2);
Nz = CONST.Ncoil(3);

Ax = CONST.Acoil(1);
Ay = CONST.Acoil(2);
Az = CONST.Acoil(3);

ix_max = CONST.i_max(1);
iy_max = CONST.i_max(2);
iz_max = CONST.i_max(3);

V_max = CONST.V_max;

% Desired current
ix = m_B(1)/(Nx*Ax);
iy = m_B(2)/(Ny*Ay);
iz = m_B(3)/(Nz*Az);

% Obtain the ratio of desire current with maximum current allowed
ratio(1) = ix_max/abs(ix);
ratio(2) = iy_max/abs(iy);
ratio(3) = iz_max/abs(iz);

% Adjust the magnetic field by proportion method of the limiting current
if min(ratio)<1
    m_B = m_B*min(ratio);
end


Wx = abs(V_max*m_B(1)/(Nx*Ax)); % [W] Power Consumption
Wy = abs(V_max*m_B(2)/(Ny*Ay)); % [W] Power Consumption
Wz = abs(V_max*m_B(3)/(Nz*Az)); % [W] Power Consumption

j  = Wx+Wy+Wz;              % [W] Total Power Consumption

% Variable Storage
global tmpVAR;
tmpVAR.moment = m_B;
tmpVAR.W = Wx+Wy+Wz;

end