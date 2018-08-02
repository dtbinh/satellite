function output = orbitROI(input)
R = input(1:3); % [m] Position Vector in Inertial Frame
V = input(4:6); % [m/s] Velocity Vector in Inertial Frame

r = norm(R);    % [m] Position Magnitude of satellite
v = norm(V);    % [m/s] Velocity Magnitude of Satellite

X = V/v;            % x-axis of orbital frame in Inertial Frame
Z = -R/r;           % z-axis of orbital frame in Inertial Frame
Y = cross(Z,X);     % y-axis or orbital frame in Inertial Frame
R_O_I = [X';Y';Z']; % Rotation Matrix from Inertial Frame to Orbital Frame

w_I_OI = cross(R,V)/(r^2);
w_O_OI = R_O_I*w_I_OI;

output = [R_O_I, w_O_OI];

end