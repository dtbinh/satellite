function output = rotation_quaternion(input)
%% INPUT
q_B_O = input(1:4); % Quaternion (xyzw) of Body from Orbital Frame (true)
q_B_I = input(5:8); % Quaternion (xyzw) of Body from Inertia Frame (true)

q_B_O = qnorm(q_B_O);% Normalisation of Quaternions(xyzw) Body Frame from Orbit Frame
q_B_I = qnorm(q_B_I); % Normalisation of Quaternions(xyzw) Body Frame from Inertia Frame
%% TRANSFORMATION MATRIX
R_B_O = q2dcm(q_B_O,'xyzw','tsf'); % Transformation Matrix of Orbit Frame to Body Frame
R_B_I = q2dcm(q_B_I,'xyzw','tsf'); % Transformation Matrix of Inertial Frame to Body Frame
R_O_B = R_B_O';
R_O_I = R_O_B*R_B_I;      % Transformation Matrix Inertial Frame to Orbit Frame


%% EULER ANGLES
e_B_I = dcm2eul(R_B_I,'zyx');  % Euler Angles for Transformation from Inertia to Body
e_B_O = dcm2eul(R_O_B','zyx'); % Euler Angles for Transformation from Orbit to Body

%% OUTPUT
output = [R_O_B, R_B_I, R_O_I, e_B_O, e_B_I];

end