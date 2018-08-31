function  S_S_m = sensor_sun(S_S)
global CONST

sig_ss = CONST.sig_ss;
FOV_ss = CONST.FOV_ss;

% True Sun Vector in Sensor Frame
S_S_m =(S_S(3,1) >= cos(FOV_ss/2))*(S_S + sig_ss*[randn(1,1);randn(1,1);randn(1,1)]);

end