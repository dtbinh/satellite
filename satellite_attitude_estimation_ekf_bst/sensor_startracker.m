function output = sensor_startracker(input)
q_B_I   = input(1:4,1); % True Rotation(wxyz) Matrix from Inertia to Body Frame
e_B_I_m = input(5:7,1); % Measured Euler Angle from Inertia to Body

%% NOISY EULER TO QUATERNION MEASUREMENTS
q_B_I_m = eul2q(e_B_I_m,'zyx','wxyz');

[~,I] = max(abs(q_B_I_m));
q_B_I_m = q_B_I_m/norm(q_B_I_m)*sign(q_B_I_m(I,1)/q_B_I(I,1));

output = [q_B_I_m;q_B_I];
end