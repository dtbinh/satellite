function output = torqueSolar(input)
global CONST;          % [] Global Constant
P_Sol = CONST.SolarP;   % [N/m^2] solar wind pressure
dx    = CONST.dx;
dy    = CONST.dy;
dz    = CONST.dz;

S_B     = input(1:3);  % [] Sun Vector in the Body Frame
eclipse = input(4);    % [] Eclipse Status


[R,A] = surfCuboid(dx,dy,dz); % Obtain the Geometry of the satellite
P_Sol = (eclipse==0)*P_Sol;
tau_s = torqsolar(R,A,S_B,P_Sol); % Torque exerted by Solar Pressure

output = tau_s;
end