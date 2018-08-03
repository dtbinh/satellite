function output = actuatorMagnetorquer(input)
m_b = input(1:3);        % [Am^2] Desired Magnetic Moment
B_b = input(4:6);        % [T] True Magnetic Vector


tau_m  = cross(m_b,B_b); % [Nm] Torque created by magnetic moment in Earth's Magnetic Field

output = tau_m;
end