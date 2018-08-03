%% Dissipative non-linear detumbling controller
function output = dissipative_controller(input)

global K_coil
global K_d

w_b_ob = input(1:3);
B_b    = input(4:6);

m_B = -(K_d/norm(B_b,2)^2)*cross(B_b,w_b_ob); % magnetic moment before scaling

% V = -(K_d/norm(B_b,2))*(K_coil^-1)*cross(B_b,w_b_ob);
V = (K_coil^-1)*m_B;
%tau_m   = cross(m_B,B_b);

output = V;
end