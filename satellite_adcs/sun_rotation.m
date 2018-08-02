function output = sun_rotation(input)
global CONST;

u = input;

R_S_I = dcm(3,u)*dcm(1,CONST.gamma);

S_eci = R_S_I'*[1;0;0];

output = S_eci;


end