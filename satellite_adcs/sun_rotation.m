function output = sun_rotation(input)
global CONST;

u = input;                           % [rad] Right ascension of the sun in the ecliptic plane

R_S_I = dcm(3,u)*dcm(1,CONST.gamma); % [rad] Rotation Matrix from Inertial Frame to Sun
S_I = R_S_I'*[1;0;0];                % [ ] Sun Vector in Inertial Frame

[r_sun_mod,rtasc_sun,decl_sun]  = jdut2sun(CONST.JD_UTC);
[r_sun_eci,v_sun_eci,a_sun_eci] = mod2eci(r_sun_mod,[0; 0; 0],[0; 0; 0],CONST.T_TT );

output = S_I;


end