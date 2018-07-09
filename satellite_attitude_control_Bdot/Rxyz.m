function R = Rxyz(phi,theta,psi)

cx = cos(phi);
sx = sin(phi);
cy  = cos(theta);
sy  = sin(theta);
cz = cos(psi);
sz = sin(psi);
 
R = [
    cy*cz           -cy*sz           sy;
    cz*sx*sy+cx*sz  cx*cz-sx*sy*sz   -cy*sx;   
   -cx*cz*sz+sx*sz  cz*sx+cx*sy*sz   cx*cy ];