function output = earth_rotation(input)
global CONST;
                          
t = input;    % [s]


GST = jdut2gst(CONST.JD_UT1  + t/60/60/CONST.sidereal);     % [rad] 
R_E_I = dcm(3,GST);

output = R_E_I;
end