function qm = qmult(q1,q2)
% q = [ x y z w]
%    q1 = first quaternion set (mx4)
%    q2 = second quaternion set (mx4)
% q2.q1
qm(1,1) =  q1(4)*q2(1)+q1(3)*q2(2)-q1(2)*q2(3)+q1(1)*q2(4);
qm(2,1) = -q1(3)*q2(1)+q1(4)*q2(2)+q1(1)*q2(3)+q1(2)*q2(4);
qm(3,1) =  q1(2)*q2(1)-q1(1)*q2(2)+q1(4)*q2(3)+q1(3)*q2(4);
qm(4,1) = -q1(1)*q2(1)-q1(2)*q2(2)-q1(3)*q2(3)+q1(4)*q2(4);


