function output = orbitROI(input)
R = input(1:3); % [m] Position Vector in Inertial Frame
V = input(4:6); % [m/s] Velocity Vector in Inertial Frame

r = norm(R);    % [m] Position Magnitude of satellite
v = norm(V);    % [m/s] Velocity Magnitude of Satellite

X = V/v;            % x-axis of orbital frame
Z = -R/r;           % z-axis of orbital frame
Y = cross(Z,X);     % y-axis or orbital frame
R_O_I = [X';Y';Z']; % Rotation Matrix from Inertial Frame to Orbital Frame



output = R_O_I;

end