function output = orbitpropagate(input)

R    = input(1:3);    % [m] Position Vector in Inertial Frame
V    = input(4:6);    % [m/s] Velocity Vector in Inertial Frame

global CONST;

mu = CONST.mu;      % [m^3/s^2]
J2 = CONST.J2;      % [-] J2 term
Re = CONST.Re;      % [m] Earth radius

% Obtain the Orbital Elements
[p,a,ecc,incl,omega,argp,nu,m] = rv2coe (R,V);

r = norm(R);        % [m] magnitude of vector R

% Perifocal Frame
P = [ -mu/r^2*3/2*J2*(Re/r)^2*(1-3*(sin(incl))^2*(sin(argp+nu))^2);
      -mu/r^2*3/2*J2*(Re/r)^2*(sin(incl))^2*sin(2*(argp+nu))   ;
      -mu/r^2*3/2*J2*(Re/r)^2*sin(2*incl)*sin(argp+nu)           ];
  
P = dcm(3,-omega)*dcm(1,-incl)*dcm(3,-(argp+nu))*P;

R_dotdot = -mu/r^3*R +P;

output = R_dotdot;
end