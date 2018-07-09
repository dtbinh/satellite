function rho = density(h)
if h <=180
    a0 =  7.001985e-02;
    a1 = -4.336216e-03;
    a2 = -5.009831e-03;
    a3 =  1.621827e-04;
    a4 = -2.471283e-06;
    a5 =  1.904383e-08;
    a6 = -7.189421e-11;
    a7 =  1.060067e-13;


    polyfn = a0 + a1*h + a2*h^2 + a3*h^3 + a4*h^4 + a5*h^5 + a6*h^6 + a7*h^7;

    rho = 10^polyfn;
else
    if h >180 && h <=1000
    
    F10 = 150; % as a proxy for the solar EUV output    
    Ap  = 0;   % geomagnetic Ap index as a proxy for the geomagnetic activity
    
    T = 900 + 2.5*(F10 - 70)+1.5*Ap; % [K]
    u = 27 - 0.012*(h-200);          % [-]
    H = T/u;                         % [km] Scale Height
    rho = 6e-10*exp(-(h-175)/H);     % [kg/m^3]
    
    else
        rho = 0.0;
    end

end



end