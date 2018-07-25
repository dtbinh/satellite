function out = bldc_hall(in)

theta_e = in;

if (theta_e < 0)
    theta_e = theta_e + 2*pi;
end

if (theta_e > -pi/3) && (theta_e <= 0) 
    stat = 1;
    hout = [1;0;1];
end

if (theta_e > 0) && (theta_e <= pi/3) 
    stat = 1;
    hout = [1;0;0];
end

if (theta_e > pi/3) && (theta_e <= 2*pi/3)  
    stat  = 2;
    hout  = [1;1;0];
end

if (theta_e > 2*pi/3) && (theta_e <= pi)  
    stat  = 3;
    hout  = [0;1;0];
end

if (theta_e > pi) && (theta_e <= 4*pi/3)  
    stat  = 4;
    hout  = [0;1;1];
end

if (theta_e > 4*pi/3) && (theta_e <= 5*pi/3)  
    stat  = 5;
    hout  = [0;0;1];
end

if (theta_e > 5*pi/3) && (theta_e <= 2*pi)  
    stat  = 6;
    hout  = [1;0;1];
end

if (theta_e > 2*pi) && (theta_e <= 7*pi/3)  
    stat  = 6;
    hout  = [1;0;1];
end

out = [hout;stat];

end