function R = Rzyx(x,y,z)
% R = Rzyx(phi,theta,psi) computes the Euler angle
% rotation matrix R in SO(3) using the zyx convention

 

R = [  cos(y)*cos(z), cos(x)*sin(z) + cos(z)*sin(x)*sin(y), sin(x)*sin(z) - cos(x)*cos(z)*sin(y);
      -cos(y)*sin(z), cos(x)*cos(z) - sin(x)*sin(y)*sin(z), cos(z)*sin(x) + cos(x)*sin(y)*sin(z);
              sin(y),                       -cos(y)*sin(x),                        cos(x)*cos(y)];
 