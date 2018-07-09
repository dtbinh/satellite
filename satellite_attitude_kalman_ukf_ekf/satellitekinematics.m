function output = satellitekinematics(input)
w_BN = input(1:3,1);
w_ON = input(4:6,1);
q_BN = input(7:10,1);
q_BO = input(11:14,1);

w_BO = w_BN-w_ON;

omega_BN = [   0     w_BN(3)  -w_BN(2) w_BN(1);
            -w_BN(3)    0      w_BN(1) w_BN(2);
             w_BN(2) -w_BN(1)    0     w_BN(3);
            -w_BN(1) -w_BN(2) -w_BN(3)    0  ];
     
     
qdot_BN = 0.5*omega_BN*q_BN;

omega_BO = [   0     w_BO(3)  -w_BO(2) w_BO(1);
            -w_BO(3)    0      w_BO(1) w_BO(2);
             w_BO(2) -w_BO(1)    0     w_BO(3);
            -w_BO(1) -w_BO(2) -w_BO(3)    0  ];
     
     
qdot_BO = 0.5*omega_BO*q_BO;

output(1:4,1) = qdot_BN;
output(5:8,1) = qdot_BO;
end