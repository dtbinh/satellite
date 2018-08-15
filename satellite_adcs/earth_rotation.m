function output = earth_rotation(input)
global CONST;
                          
t = input;    % [s]

model = 1;

switch model
    case 0
        % Simple Model
        R_I_E = dcm(3,-CONST.w_earth(3)*t);
        
        r_tgt = [0;0;0]; % TODO
        
    case 1
        % Greenwich sidereal Time Model
        GST = jdut2gst(CONST.JD_UT1  + t/60/60/CONST.sidereal);    % [rad]    
        R_E_I = dcm(3,GST);
        R_I_E = R_E_I';
        
        r_tgt = latlon2vec(CONST.lat,CONST.lon,rad2deg(GST)); 
        
        
    otherwise
        fprintf('earth_rotation() error');
end 


output = [R_I_E r_tgt];
end