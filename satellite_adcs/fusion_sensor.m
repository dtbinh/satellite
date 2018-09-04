function output = fusion_sensor(input)
B_B_m = input(1:3,1);
S_B_m = input(1:3,2);

B_I   = input(1:3,3);
S_I   = input(1:3,4);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% bst_attx_det_make_from_2vec
if (S_B_m)
    u1 = vnorm(B_I);
    u2 = vnorm(S_I);

    b1 = vnorm(B_B_m);
    b2 = vnorm(S_B_m);
else
    u1 = vnorm(B_I);
    u2 = vnorm(B_I);

    b1 = vnorm(B_B_m);
    b2 = vnorm(B_B_m);
end

v10 = dot(u1,u2);
vi1p0_1 = u1;   
vi1p0_2 = u2 - v10 * u1;            % Perpendicular to u1 in direction close to u2
vi1p0_2 = vnorm(vi1p0_2);


v10b = dot(b1,b2);
vb1p0_1 = b1;
vb1p0_2 = b2 - v10b * b1 ;           % Perpendicular to u1 in direction close to u2
vb1p0_2 = vnorm(vb1p0_2);


[R_B_I_m,q_B_I_m] = quest(vb1p0_1,vb1p0_2,vi1p0_1,vi1p0_2,'crassidis');

output = q_B_I_m;

end