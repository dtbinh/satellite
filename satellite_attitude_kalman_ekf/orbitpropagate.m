function output = orbitpropagate(input)

global CONST;
mu = CONST.mu;      % [m^3/s^2]
R  = input(1:3);    % [m] Position Vector in Inertial Frame

R_dotdot = -mu/((norm(R))^3)*R;

output = R_dotdot;


end