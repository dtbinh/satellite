function output = transformation(input)
q_BN = input(1:4,1);
q_BO = input(5:8,1);

EulerBO321 = Quat2Euler(q_BO);
EulerBN321 = Quat2Euler(q_BN);

C_BO = q2R(q_BO,'xyzw','tsf');
C_NB = q2R(q_BN,'xyzw','rot');


output(1:3,1:3) = C_BO;
output(1:3,4)   = EulerBO321;
output(1:3,5)   = EulerBN321;
output(1:3,6:8) = C_NB;