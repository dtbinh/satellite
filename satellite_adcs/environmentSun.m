function output = environmentSun(input)

R     = input(1:3,1);   % [m] Satellite Position Vector in Inertial Frame
R_O_I = input(1:3,2:4); % [-] Rotation Matrix from Inertial Frame to Orbit Frame
R_B_I = input(1:3,5:7); % [-] Rotation Matrix from Inertial Frame to Body Frame
TA    = input(1,8);
beta  = input(2,8);

%% GLOBAL PARAMETERS
global CONST;

Re  = CONST.Re;         % [m] Earth Radius
rho = asin(Re/norm(R)); % [rad] Earth Angular Radius  

R_I_B = R_B_I';         % [-] Rotation Matrix from Body Frame to Inertial Frame
R_O_B = R_O_I*R_I_B;    % [-] Rotation Matrix from Body Frame to Orbital Frame
R_B_O = R_O_B';         % [-] Rotation Matrix from Orbital Frame to Body Frame

S_B = R_B_O*[cos(beta)*sin(TA);sin(beta);cos(beta)*cos(TA)]; % Sun Direction Vector in Body Frame
S_I = R_I_B*S_B;                                             % Sun Direction Vector in Inertia Frame

if cos(TA) + cos(rho)/cos(beta) < 0 
    eclipse = [1;1;1];
else
    eclipse = [0;0;0];
end

output = [S_B S_I eclipse];

end