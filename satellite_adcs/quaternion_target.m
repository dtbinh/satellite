function output = quaternion_target(input)
R     = input(1:3,1);
V     = input(1:3,2);
R_tgt = input(1:3,3);

global CONST

[R_i_r,r1,r2,r3] = triad2dcm(R_tgt-R, V);

R_r_i = R_i_r';


R_b_r = triad2dcm(CONST.los_vec,CONST.vel_vec);

R_b_i = R_b_r*R_r_i;
R_i_b = R_b_i';

q_tgt = dcm2q(R_i_b);


output = q_tgt;
end
    