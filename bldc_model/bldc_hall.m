function out = bldc_hall(in)

theta_e = in;

h1 = 0;
h2 = 0;
h3 = 0;

if (theta_e >= -pi/3) && (theta_e <= 2*pi/3) 
    h1 = 1;
end

if (theta_e >= pi/3) || (theta_e <= -2*pi/3)  
    h2 = 1;
end

if (theta_e >= -pi) && (theta_e <= 0) 
    h3 = 1;
end


out = [h1;h2;h3];

end