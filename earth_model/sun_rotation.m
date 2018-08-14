function output = sun_rotation(input)
global CONST;

                           
t = input;     % [s]

r_sun_mod = jdut2sun(CONST.JD_UT1  + t/60/60/CONST.sidereal);
r_sun_eci = mod2eci(r_sun_mod,[0; 0; 0],[0; 0; 0],CONST.T_TT );



output = r_sun_eci;


end