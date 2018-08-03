function output = environmentDensity(input)
%% INPUT
R    = input(1:3); % [m]   Satellite Position with reference to Earth Center in Inertial Frame
TA   = input(4);   % [rad] True Anomaly Angle between Ascending Node and Satellite
beta = input(5);   % [rad] Sun Beta Angle between Orbit Plane and Sun

global CONST;

h   = (norm(R) - CONST.Re)/1000;   % [km] Height above Earth Surface 
rho = density(h);               % [kg/m^3] Density of Atmosphere as function of height
rho = rho*1.5^(cos(beta)*cos(TA)); % [kg/m^3] Density of Atmosphere as function of beta and TA
CONST.rho = rho;
output = rho;
end