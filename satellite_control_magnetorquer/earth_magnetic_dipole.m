function output = earth_magnetic_dipole(input)
lat_O = input(1);

global B_null

B_O = B_null*[  cos(lat_O)  ;
                    0       ;
                2*sin(lat_O)];
output = B_O;

end