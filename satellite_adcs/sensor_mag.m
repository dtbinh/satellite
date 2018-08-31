function  B_B_m = sensor_mag(B_B)
global CONST

sig_mg = CONST.sig_mg;
Umg    = CONST.Umg;

% Measured Magnetic Field in Body Frame
B_B_m = (Umg*B_B + sig_mg*[randn(1,1);randn(1,1);randn(1,1)]);

end