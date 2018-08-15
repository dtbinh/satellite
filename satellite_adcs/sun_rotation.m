function output = sun_rotation(input)
global CONST;

u = input;                           % [rad] Right ascension of the sun in the ecliptic plane
t = get_param('satellite_adcs_model','SimulationTime');

model = 1;

switch model
    case 0
        
        R_S_I = dcm(3,u)*dcm(1,CONST.gamma); % [rad] Rotation Matrix from Inertial Frame to Sun
        S_I = R_S_I'*[1;0;0];                % [ ] Sun Vector in Inertial Frame

    case 1
        r_sun_mod = jdut2sun(CONST.JD_UTC + t/60/60/CONST.sidereal);
        S_I = mod2eci(r_sun_mod,[0; 0; 0],[0; 0; 0],CONST.T_TT );


end

output = S_I;
end