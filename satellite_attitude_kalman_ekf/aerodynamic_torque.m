function output = aerodynamic_torque(input)

R_B_I   = input(1:3,1:3); % [-]   Rotation Matrix from Inertial to Body Frame
V_I     = input(1:3,4);   % [m/s] Velocity Vector of Satellite in Inertial Frame
w_B_BI  = input(1:3,5);   % [rad/s] Angular Velocity of Satellite Body relative to Inertial Frame in Body Frame
rho     = input(1,6);     % [kg/m^3] Density of Atmosphere

V_B = R_B_I*V_I;          % [m/s] Velocity of Satellite in Body Frame 


[R,A] = sat_geo('2u'); % Obtain the Geometry of the satellite

% tau_a = aero_torque(R,A,V,w,rho,'hughes')
tau_a = aero_torque(R,A,V_B,w_B_BI,rho,'wertz');


output = tau_a;
end