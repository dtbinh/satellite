function output = sunDynamics(input)
%% GLOBAL PARAMETERS
global CONST;

gamma = CONST.gamma; % [rad] Spin Axis of Ecliptic Plane

%% INPUT 
R   = input(1:3); % [m] Position Vector of Satellite in Inertia Frame
V   = input(4:6); % [m] Veloctiy Vector of Satellite in Inertia Frame
u   = input(7);   % [rad] Right ascension of the sun in the ecliptic plane

R_S_I = DCM(3,u)*DCM(1,gamma); % [-] Transformation Matrix from Inertia to Sun Frame

S_I = R_S_I'*[1;0;0]; % [-] Sun Vector in Inertia Frame        

%% TRUE ANOMALY FROM SUN-ORBIT AXIS
H   = cross(R,V);                    % [m^2/s] Angular Momentum Vector Z axis of Perifocal Frame
S_X = cross(S_I,H);
S_Y = cross(H,S_X);
S_Y = S_Y/norm(S_Y);

S_TA = acos(dot(S_Y,R)/norm(S_Y)/norm(R));
if acos(dot(-S_X,R)/norm(-S_X)/norm(R))>pi/2
    S_TA = 2*pi-S_TA;
end

output(1:3,1) = S_I;  % Sun vector in Inertia Frame
output(4,1)   = S_TA; % True anomaly from Sun-orbit Axis

end