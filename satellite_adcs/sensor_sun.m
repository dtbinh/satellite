function  output = sensor_sun(input)

S_B = input(1:3,1); % True Sun Vector in Body Frame

global CONST

sig_ss = CONST.sig_ss;

S_S = CONST.R_S1_B*S_B; % True Sun Vector in Sensor Frame

S_S_m = S_S + sig_ss*[randn(1,1);randn(1,1);randn(1,1)];
cps(FOV_ss/2)

output = S_S_m;

end