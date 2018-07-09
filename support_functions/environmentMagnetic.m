function output = environmentMagnetic(input)
%% GLOBAL VARIABLES
global CONST

%% INPUT
R_I_E = input(1:3,1:3); % Rotation Matrix from Earth Frame to Inertial
R     = input(1:3,4);   % Position Vector of Satellite in Inertia Frame

%%
R_E_M = dcm(1,-11.5/180*pi); % Rotation Matrix from Magnetic Frame to Earth Frame
R_E_I = R_I_E';              % Rotation Matrix from Inertia Frame to Earth Frame

%% POSITIVE VECTOR IN MAGNETIC FRAME
R_ecf = R_E_I*R;             % Position of Satellite in Earth Fixed Frame
R_emf = R_E_M'*R_ecf;        % Position of Satellite in Earth Magnetic Frame

%% SPHERE COORDINATES IN EARTH MAGNETIC FRAME

R_emf_sph(1,1) = sqrt(R_emf(1)^2+R_emf(2)^2+R_emf(3)^2); % [rad] Radius from Center of Earth
R_emf_sph(2,1) = acos(R_emf(3)/norm(R_emf));             % [rad] theta from Z axis of Magnetic Frame
R_emf_sph(3,1) = atan2(R_emf(2),R_emf(1));               % [rad] phi   from X axis of Magnetic Frame

%% DIPOLE MAGNETIC FIELD
B_sph(1,1) = -2*CONST.Bo/((R_emf_sph(1))^3)*cos(R_emf_sph(2)); % [T] Magnetic Field radius component
B_sph(2,1) =   -CONST.Bo/((R_emf_sph(1))^3)*sin(R_emf_sph(2)); % [T] Magnetic Field theta component
B_sph(3,1) = 0;                                                % [T] Magnetic Field phi component

R_C_S = Rcart('sphere',R_emf_sph(3,1),R_emf_sph(2,1)); % [-] Rotation Matrix from Sphere to Cartesian (Magnetic Frame)

B_emf = R_C_S*B_sph;                                   % [T] Earth Magnetic Field Vector in Magnetic Dipole Frame 
B_ecf = R_E_M*B_emf;                                   % [T] Earth Magnetic Field Vector in Earth Center Fixed Frame 
B_eci = R_I_E*B_ecf;                                   % [T] Earth Magnetic Field Vector in Earth Center Inertial Frame 

output = B_eci;   

end