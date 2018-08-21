function output = quaternion_target(input)
R     = input(1:3,1);
V     = input(1:3,2);
R_tgt = input(1:3,3);

global CONST

persistent q_B_I_tgt_prev qd_B_I_tgt_prev w_B_BI_tgt_prev

if isempty(q_B_I_tgt_prev)
    q_B_I_tgt_prev = [0;0;0;1];
    qd_B_I_tgt_prev = [0;0;0;1];
    w_B_BI_tgt_prev    = [0;0;0];
end 

[R_i_r,r1,r2,r3] = triad2dcm(R_tgt-R, V);

R_r_i = R_i_r';


R_b_r = triad2dcm(CONST.los_vec,CONST.vel_vec);
R_b_i = R_b_r*R_r_i;

% Quaternion Target
q_B_I_tgt = dcm2q(R_b_i);

q_B_I_tgt = qchk(q_B_I_tgt, q_B_I_tgt_prev);

% Angular Velocity Target
% w_B_BI_tgt3  = 2/norm(q_B_I_tgt)^2*(q2xi(q_B_I_tgt))'*qd_B_I_tgt; % John
qd_B_I_tgt = (q_B_I_tgt - q_B_I_tgt_prev)/CONST.dt;
temp = qmul(qd_B_I_tgt,qinvert(q_B_I_tgt));
w_B_BI_tgt  = 2*(temp(1:3,1));

% Angular Acceleration
qdd_B_I_tgt = (qd_B_I_tgt - qd_B_I_tgt_prev)/CONST.dt;
temp = qmul(qdd_B_I_tgt,qinvert(q_B_I_tgt))+qmul(qd_B_I_tgt,qinvert(qd_B_I_tgt));
wd_B_BI_tgt = 2*(temp(1:3,1));

wd_B_BI_tgt = (w_B_BI_tgt - w_B_BI_tgt_prev)/CONST.dt;

output = [q_B_I_tgt;w_B_BI_tgt;wd_B_BI_tgt];


q_B_I_tgt_prev  = q_B_I_tgt;
qd_B_I_tgt_prev = qd_B_I_tgt;
w_B_BI_tgt_prev = w_B_BI_tgt;
end
    