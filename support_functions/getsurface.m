function [vertices_x,vertices_y ,vertices_z ] = getsurface(R,N,dA)
% This function get the vertices of the input R,N,dA

n = size(R,2); 

for i = 1:n
qrot = vrrotvec([0;0;1],N(:,i)); % calculates a rotation needed to transform the 3D vector a to the 3D vector b.
dcm  = vrrotvec2mat(qrot);
 
body_vertices(1,:) = R(:,i)+ dcm*[dA(1,i)/2;dA(2,i)/2;dA(3,i)];
body_vertices(2,:) = R(:,i)+ dcm*[-dA(1,i)/2;dA(2,i)/2;dA(3,i)];
body_vertices(3,:) = R(:,i)+ dcm*[-dA(1,i)/2;-dA(2,i)/2;dA(3,i)];
body_vertices(4,:) = R(:,i)+ dcm*[dA(1,i)/2;-dA(2,i)/2;dA(3,i)];

vertices_x(:,i) = body_vertices(:,1);
vertices_y(:,i) = body_vertices(:,2);
vertices_z(:,i) = body_vertices(:,3);
end
 end
