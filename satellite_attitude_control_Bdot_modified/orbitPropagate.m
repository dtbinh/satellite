function output = orbitPropagate(input)
%% GLOBAL PARAMETERS
global CONST;

mu = CONST.mu;      % [m^3/s^2]

%% INPUT 
R  = input(1:3);    % [m] Position Vector of Satellite in Inertial Frame

%% ORBIT PROPAGATION
R_dotdot = -mu/((norm(R))^3)*R;

output = R_dotdot;


end