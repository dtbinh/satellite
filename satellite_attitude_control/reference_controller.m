%% reference controller
function output = reference_controller(input)
global K_coil
global K_d
global K_p
%% INPUT PARAMETERS
w_b_ob = input(1:3);
B_b    = input(4:6);
q      = input(7:10);

%% FUNCTION
eps = q(2:4);

m_B = -(K_d/norm(B_b,2)^2)*cross(B_b,w_b_ob)-(K_p/norm(B_b,2)^2)*cross(B_b,eps);
V   =  (K_coil^-1)*m_B;

output = V;
end