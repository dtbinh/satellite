function output = rotation_quaternion(input)
%% INPUT
R_B_I = input(1:3,1:3); % Transformation Matrix Inertial Frame to Body Frame
R_O_I = input(1:3,4:6); % Transformation Matrix Inertial Frame to Orbit Frame

%% TRANSFORMATION MATRIX
R_I_B = R_B_I';
R_O_B = R_O_I*R_I_B;
R_B_O = R_O_B';

%% EULER ANGLES
e_B_I = dcm2eul(R_B_I,'zyx');  % Euler Angles for Transformation from Inertia to Body
e_B_O = dcm2eul(R_B_O,'zyx'); % Euler Angles for Transformation from Orbit to Body


%% OUTPUT
output = [R_O_B, R_B_I, R_O_I, e_B_O, e_B_I, R_B_O];

end