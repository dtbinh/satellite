function output = earth_rotation(input)
global CONST;
                          
t = input;    % [s]

earthmodel = 1;

%% TRANSFORMATION MATRIX 
switch earthmodel
    case 0
        % Simple Model
        R_I_E = dcm(3,-CONST.w_earth(3)*t);
        
    case 1
        % Greenwich sidereal Time Model
        GST = jdut2gst(CONST.JD_UT1  + t/60/60/CONST.sidereal);      
        R_E_I = dcm(3,GST);
        R_I_E = R_E_I';
        
    otherwise
        fprintf('earth_rotation() error');
end 

output = R_I_E;
end