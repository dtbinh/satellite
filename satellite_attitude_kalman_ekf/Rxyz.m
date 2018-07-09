function R = Rxyz(x,y,z)
 


R=[                       cos(y)*cos(z),                        cos(y)*sin(z),       -sin(y);
   cos(z)*sin(x)*sin(y) - cos(x)*sin(z), cos(x)*cos(z) + sin(x)*sin(y)*sin(z), cos(y)*sin(x);
   sin(x)*sin(z) + cos(x)*cos(z)*sin(y), cos(x)*sin(y)*sin(z) - cos(z)*sin(x), cos(x)*cos(y)];