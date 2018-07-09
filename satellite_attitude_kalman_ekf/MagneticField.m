function MagneticField()
dm         = 7.94e22;      % [Am^2] Earth Dipole Magnetic Moment
mui        = 4*pi*1e-7;    % [kgm/A^2/s^2] Earth Permeability of Free Space
Re         = 6.378137E6;        % [m] earth radius
B0         = mui*dm/4/pi;   
i = 1;

for rad = 0:7500e3/9:7500e3
for lon = 0:2*pi/18:2*pi
for lat = -pi/2:pi/18:pi/2
    
    theta = pi/2-lat;
    phi   = lon;
    
    % spherical
    B_sph(1,i) = -2*B0/((rad)^3)*cos(theta);
    B_sph(2,i) = -B0/((rad)^3)*sin(theta);
    B_sph(3,i) = 0;
   
    R_C_S(:,:,i) = Rcart('sphere',phi,theta);

    B_xyz(:,i) = R_C_S(:,:,i)*B_sph(:,i);   % [T] Earth Magnetic Field Vector in Magnetic Dipole Frame     
    B_emf(:,i) = DCM(1,11.5/180*pi)*B_xyz(:,i);

    % Normalising
    B(1) = B_emf(1,i)/norm(B_emf(:,i))*0.1 ;
    B(2) = B_emf(2,i)/norm(B_emf(:,i))*0.1 ;
    B(3) = B_emf(3,i)/norm(B_emf(:,i))*0.1 ;
    
    % Position
    R(1) = rad*cos(lat)*cos(lon)/Re;
    R(2) = rad*cos(lat)*sin(lon)/Re;
    R(3) = rad*sin(lat)/Re;

    %Plot
    plot3([R(1) B(1)+R(1)],[R(2) B(2)+R(2)], [R(3) B(3)+R(3)],'Color',[0.7 0.7 0.7]);
    hold on;
 
    i= i+1;
end
end
end


end