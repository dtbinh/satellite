function output = rotationquaternion(input)
global w_O_IO

q      = input(1:4);
w_B_IB = input(5:7);

q = q./norm(q);

eta = q(1);
eps = q(2:4);



S_eps = [     0    -eps(3)   eps(2);
          eps(3)       0    -eps(1);
         -eps(2)   eps(1)         0 ];

R_O_B = eye(3) + 2*eta*S_eps + 2*S_eps^2;
R_B_O = R_O_B';

w_B_OB = w_B_IB - R_B_O*w_O_IO;

output = [R_B_O w_B_OB];

end