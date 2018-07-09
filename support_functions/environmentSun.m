function output = environmentSun(input)
%% GLOBAL PARAMETERS
global CONST;

Re = CONST.Re; % [m] Earth Radius

%% INPUT 
R     = input(1:3,1);
R_O_B = input(1:3,2:4);
R_B_I = input(1:3,5:7);
TA    = input(1,8);
beta  = input(2,8);

r = norm(R);      % [m] Distance from satellite to center of Earth
rho = asin(Re/r); % [rad] Earth Angular Radius  

R_B_O = R_O_B'; % [-] Rotation Matrix from Orbital Frame to Body Frame
R_I_B = R_B_I'; % [-] Rotation Matrix from Body Frame to Inertial Frame

S_B_hat = R_B_O*[cos(beta)*sin(TA);sin(beta);cos(beta)*cos(TA)]; % Sun Direction Vector
S_I_hat = R_I_B*S_B_hat;

if cos(TA) + cos(rho)/cos(beta) < 0 
    eclipse = [1;1;1];
else
    eclipse = [0;0;0];
end

output = [S_B_hat S_I_hat eclipse];

end