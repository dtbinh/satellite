function output = transformationquaternion(input)
q      = input(1:4);
w_B_OB = input(5:7);

q = q./norm(q);

eta = q(1);
eps = q(2:4);

S_eps = [     0    -eps(3)   eps(2);
      eps(3)       0    -eps(1);
     -eps(2)   eps(1)         0 ];

eta_dot = -0.5*eps'*w_B_OB;
eps_dot =  0.5*(eta*eye(3)+S_eps)*w_B_OB;

%% QUATERNION MATRIX PROPAGATION
% w_bi = w_B_OB;
% Q = [eta;eps];
% omega = [0         w_bi(3)    -w_bi(2)  w_bi(1);
%          -w_bi(3)  0           w_bi(1)  w_bi(2);
%           w_bi(2)  -w_bi(1)    0        w_bi(3);
%          -w_bi(1)  -w_bi(2)   -w_bi(3)  0   ;];
% 
% Q_dot = 0.5*omega*Q;

q_dot = [eta_dot;eps_dot];

output = q_dot;

end