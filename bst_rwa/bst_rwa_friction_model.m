function friction = bst_rwa_friction_model(spd_rpm)
frict_a = 5.69935E-4;                      % [Nm] 
frict_b = 2.06636E-7;            % [Nm/rpm]
frict_c = 1.58979E-11; % [Nm/rpm]

friction = -sign(spd_rpm)*(frict_c*(abs(spd_rpm))^2  + frict_b*abs(spd_rpm) + frict_a);
     

end