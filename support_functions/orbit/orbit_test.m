a     = (500+6378)*1000;
incl  =20*deg2rad
ecc   = 0.2
omega = 0
TA    = pi/3
argp  = 0

[r, v] = coe2rv(a, ecc,incl,omega,argp,TA,'curtis') 
[r,v]  = coe2rv (a,ecc,incl,omega,argp,TA,'vallado')