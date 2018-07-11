function q = euler2q(phi,theta,psi)

R        = Rzyx(phi,theta,psi);
R(4,4)   = trace(R);
[Rmax,i] = max( [R(1,1) R(2,2) R(3,3) R(4,4)] );
p_i      = sqrt(1+2*R(i,i)-R(4,4));

if i==1,
   p1 = p_i;
   p2 = (R(2,1)+R(1,2))/p_i;
   p3 = (R(1,3)+R(3,1))/p_i;
   p4 = (R(3,2)-R(2,3))/p_i;
elseif i==2,
   p1 = (R(2,1)+R(1,2))/p_i;
   p2 = p_i;
   p3 = (R(3,2)+R(2,3))/p_i;
   p4 = (R(1,3)-R(3,1))/p_i;
elseif i==3,
   p1 = (R(1,3)+R(3,1))/p_i;
   p2 = (R(3,2)+R(2,3))/p_i;   
   p3 = p_i;
   p4 = (R(2,1)-R(1,2))/p_i;   
else
   p1 = (R(3,2)-R(2,3))/p_i;
   p2 = (R(1,3)-R(3,1))/p_i;
   p3 = (R(2,1)-R(1,2))/p_i;   
   p4 = p_i;
end

q = 0.5*[p4 p1 p2 p3]';
q = q/(q'*q);
end