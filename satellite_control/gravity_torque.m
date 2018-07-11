function output = gravity_torque(input)
global w_O
global I

R_B_O_3 = input(1:3);


tau_gravity = 3*w_O^2*cross(R_B_O_3 ,I*R_B_O_3);

output = tau_gravity;
end