function output = earthDynamics(input)
%% MODEL TIME
t = input;

%% GLOBAL PARAMETERS
global CONST

w_E = CONST.w_earth;

%% TRANSFORMATION MATRIX 
R_I_E = DCM(3,-w_E(3)*t);

output = R_I_E;


end