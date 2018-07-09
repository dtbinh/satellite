function [R,N,dA,A] = satbody(dx,dy,dz)
% dx [m] - length of x
% dy [m] - length of y
% dz [m] - length of z

% R  [m]    - area vector
% N  [m]    - unit area vector
% dA [m^2]  - area size
% A  [m^2]  - area   
       
R(:,1)  = [dx/2;0;0];
N(:,1)  = R(:,1)/norm(R(:,1));
dA(:,1) = [dz;dy;0];
A(:,1)  = dA(1,1)*dA(2,1)*N(:,1);

R(:,2)  = [-dx/2;0;0];
N(:,2)  = R(:,2)/norm(R(:,2));
dA(:,2) = [dz;dy;0];
A(:,2)  = dA(1,2)*dA(2,2)*N(:,2);

R(:,3)  = [0;dy/2;0];
N(:,3)  = R(:,3)/norm(R(:,3));
dA(:,3) = [dx;dz;0];
A(:,3)  = dA(1,3)*dA(2,3)*N(:,3);

R(:,4)  = [0;-dy/2;0];
N(:,4)  = R(:,4)/norm(R(:,4));
dA(:,4) = [dx;dz;0];
A(:,4)  = dA(1,4)*dA(2,4)*N(:,4);
 
R(:,5) = [0;0;dz/2];
N(:,5) = R(:,5)/norm(R(:,5));
dA(:,5) = [dx;dy;0];
A(:,5) = dA(1,5)*dA(2,5)*N(:,5);
 
R(:,6) = [0;0;-dz/2];
N(:,6) = R(:,6)/norm(R(:,6));
dA(:,6) = [dx;dy;0];
A(:,6) = dA(1,6)*dA(2,6)*N(:,6);

end