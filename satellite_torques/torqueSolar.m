function output = torqueSolar(input)
S_B     = input(1:3);  % [] Sun Vector in the Body Frame
eclipse = input(4);    % [] Eclipse Status
global CONST;          % [] Global Constant
solarP = CONST.SolarP; % [N/m^2] solar wind pressure

[R,A] = sat_geo('2u'); % Obtain the Geometry of the satellite



output = tau_s;
end