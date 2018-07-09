function output = rotation_quaternion(input)
%% INPUT
q_B_O = input(1:4); % Rotation Quaternion of Body from Orbital Frame  (true)
q_B_I = input(5:8); % Rotation Quaternion of Body from Inertia Frame (true)

q_B_O   = q_B_O./norm(q_B_O); % Normalisation of Quaternions(wxyz) Body Frame from Orbit Frame
q_B_I   = q_B_I./norm(q_B_I); % Normalisation of Quaternions(wxyz) Body Frame from Inertia Frame

%% TRANSFORMATION MATRIX
R_B_O = q2R(q_B_O,'wxyz','tsf'); % Transformation Matrix of Orbit Frame to Body Frame
R_B_I = q2R(q_B_I,'wxyz','tsf'); % Transformation Matrix of Inertial Frame to Body Frame
R_O_B = R_B_O';
R_O_I = R_O_B*R_B_I;      % Transformation Matrix Inertial Frame to Orbit Frame


%% EULER ANGLES
e_B_I = R2eul(R_B_I,'ZYX');  % Euler Angles for Transformation from Inertia to Body
e_B_O = R2eul(R_O_B','ZYX'); % Euler Angles for Transformation from Orbit to Body

%% OUTPUT
output = [R_O_B, R_B_I, R_O_I, e_B_O, e_B_I];

end