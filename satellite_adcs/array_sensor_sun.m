function  S_B_m = array_sensor_sun(S_B)
global CONST

R_S_B = CONST.R_S_B;
FOV_ss = CONST.FOV_ss;
SSnum  = CONST.SSnum;


% Measured Sun Vectors in Sensor Frame
S_S_m = zeros(3,SSnum);
for i = 1:1:SSnum
S_S = R_S_B(:,:,i)*S_B;
S_S_m(:,i) = sensor_sun(S_S);
end

% Selected Measured Sun Vector in Body Frame
S_B_m = zeros(3,1);
for i = 1:1:SSnum

if (S_S_m(3,i) > cos(FOV_ss/2))
    S_B_m = R_S_B(:,:,i)'*S_S_m(:,i);
end

end