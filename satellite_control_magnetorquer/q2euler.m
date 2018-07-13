function [phi,theta,psi] = q2euler(q)
 

R = Rquat(q);
if abs(R(3,1))>1.0
    error('solution is singular for theta = +- 90 degrees');
end
 
phi   =  atan2(R(3,2),R(3,3));
theta = -asin(R(3,1));
psi   =  atan2(R(2,1),R(1,1));

end