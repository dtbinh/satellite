function R = dcm2cart(type,phi,theta)
    switch type
        case 'sphere'
        R = [sin(theta)*cos(phi) cos(theta)*cos(phi) -sin(phi);
             sin(theta)*sin(phi) cos(theta)*sin(phi)  cos(phi);
             cos(theta)        -sin(theta)         0    ];

        case 'latlon'
            lat = theta;
            lon = phi;
            
        R = [cos(lat)*cos(lon)  -sin(lat)*cos(lon)  -sin(lon);
             cos(lat)*sin(lon)  -sin(lat)*sin(lon)   cos(lon);
                   sin(lat)        cos(lat)             0];

        otherwise

    end
end