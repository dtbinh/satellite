function output = sun_sensor(input)
R_B_I = input(1:3,1:3);
e_B_I_m = input(1:3,4);

e_B_I   = R2eul(R_B_I,'ZYX');
q_B_I   = euler2q(e_B_I(1),e_B_I(2),e_B_I(3)); 
q_B_I_m = euler2q(e_B_I_m(1),e_B_I_m(2),e_B_I_m(3));


% R_q = Rquat(q_B_I);
% R_e = Rquat(q_B_I_m);
% 
% Z_q = R_q*[0;0;1];
% Z_e = R_e*[0;0;1];

output = [q_B_I_m;q_B_I];
end