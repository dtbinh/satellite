function output = magnetic_coil(input)
V   = input(1:3);
B_b = input(4:6);

global K_coil

S_B    = Smtrx(-B_b);
m_b    = K_coil*V;       % Magnetic moment from the coil
tau_m  = cross(m_b,B_b); % Torque created by magnetic moment in Earth's Magnetic Field

output = tau_m;
end