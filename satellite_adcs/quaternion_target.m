function output = quaternion_target(input)
R     = input(1:3,1);
V     = input(1:3,2);
R_tgt = input(1:3,3);

global CONST

persistent q_B_I_tgt_prev

if isempty(q_B_I_tgt_prev)
    q_B_I_tgt_prev = [0;0;0;1];
end 

[R_i_r,r1,r2,r3] = triad2dcm(R_tgt-R, V);

R_r_i = R_i_r';


R_b_r = triad2dcm(CONST.los_vec,CONST.vel_vec);

R_b_i = R_b_r*R_r_i;

q_B_I_tgt = dcm2q(R_b_i);

qd_B_I_tgt = (q_B_I_tgt - q_B_I_tgt_prev)/CONST.dt;
temp = qmul(qd_B_I_tgt,qinvert(q_B_I_tgt));
w_B_BI_tgt  = 2*(temp(1:3,1));

q_B_I_tgt_prev = q_B_I_tgt;

output = [q_B_I_tgt;w_B_BI_tgt];
end
    