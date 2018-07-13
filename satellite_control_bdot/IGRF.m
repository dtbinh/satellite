function B = IGRF(n,m,lat,lon,P)

load('IGRF11_data')

i = n;
j = m;

% Redefining Latitude in case of Possible Singularity
mt  = mod(lat,2*pi);
tol = 1e-9;

if (mt>(pi/2)-tol && mt<(pi/2)+tol)
    lat = (pi/2)+tol;
elseif (mt >(3*pi/2)-tol && mt<(3*pi/2)+tol)
    lat = (3*pi/2)+tol;
end

% Define the Constants
Re = P.Re;
Rc = P.Rc;

% Calculating Colatitude and Longitude in Radians
theta = (pi/2)-lat;
phi   = lon;

% Zero Offset
O = 1;

% Defining temporary variables
Bt2 = 0;
Bp2 = 0;
Br2 = 0;

% Calculating Legendre Polynominals
[P,dP,S] = Pfunk(n,m,theta);

% Calculating Field Vectors
for n = 1:i
    Bt1 = 0;
    Bp1 = 0;
    Br1 = 0;
    
    for m = 0:j
        Br1 = Br1 + (  S(O+n,O+m)*g_data(O+n,O+m)*cos(m*phi)+  S(O+n,O+m)*h_data(O+n,O+m)*sin(m*phi))*P(O+n,O+m);
        Bt1 = Bt1 + (  S(O+n,O+m)*g_data(O+n,O+m)*cos(m*phi)+  S(O+n,O+m)*h_data(O+n,O+m)*sin(m*phi))*dP(O+n,O+m);
        Bp1 = Bp1 + (m*S(O+n,O+m)*h_data(O+n,O+m)*cos(m*phi)-m*S(O+n,O+m)*g_data(O+n,O+m)*sin(m*phi))*P(O+n,O+m);
        
    end
    
    Bt2 = Bt2 + ((Re/Rc)^(n+2))*Bt1;
    Bp2 = Bp2 + ((Re/Rc)^(n+2))*Bp1;
    Br2 = Br2 + (n+1)*((Re/Rc)^(n+2))*Br1;
    


end

% In Cartesian Coordinates
eps = 0; % Earth's Oblateness Error Term, typically less than 0.2 degree

X   =  Bt2*cos(eps) - Br2*sin(eps);
Y   = -Bp2/(sin(theta));
Z   = -Bt2*sin(eps)-Br2*cos(eps);

B = [X;Y;Z]*1e-9;

end