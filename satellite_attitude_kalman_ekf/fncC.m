function output = fncC(input)
q   = input(1:4,1);
B_I = input(5:7,1);

q = qinvert(q,'wxyz');
R_B_I  = q2xi(q)'*q2psi(q);

    output = R_B_I*B_I; 