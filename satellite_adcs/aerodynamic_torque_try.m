%% MODELLING OF AERODYNAMICS TORQUE 
close all
clear
clc
%% SPACECRAFT PARAMETERS

dz = 0.2; % [m] length of z
dy = 0.1; % [m] length of y
dx = 0.1; % [m] length of x

% Area 1 ( Top)
R1 = [0;0;-dz/2]
A1 = [0;0;-dx*dy]

% Area 2 (Bottom) 
R2 = [0;0;dz/2]
A2 = [0;0;-dx*dy]

% Area 3 (Side 1)

r1 = [ 0.7071;
      -0.7071;
          0  ];
         
r2 = [-0.7071;
       0.7071;
          0  ];      
% Area Surface Vectors
A1 = 2;
A2 = 2;

N1 =  [-0.7071;
       -0.7071;
           0];
N2 =  [-0.7071;
       -0.7071;
           0];

Vo =[-1;0;0 ]; % [] Unit Vector of Velocity
V = 5;  % [] Velocity Magnitude

w = [0;0;1];

rho = 0.5;     % [] Density

%% WERTZ SPACECRAFT ATTITUDE DETERMINATION AND CONTROL
wertz_term1_area1 = dot(N1,Vo)*(cross(Vo,r1))
wertz_term1_area2 = dot(N2,Vo)*(cross(Vo,r2))

wertz_term2_area1 = dot(N1,cross(w,r1))*cross(Vo,r1)
wertz_term2_area2 = dot(N2,cross(w,r2))*cross(Vo,r2)

wertz_term3_area1 = dot(N1,Vo)*(cross(cross(w,r1),r1))
wertz_term3_area2 = dot(N2,Vo)*(cross(cross(w,r2),r2))

%% HUGHES SPACECRAFT ATTUTDE DYNAMICS
hughes_term1_area1 = dot(N1,Vo)*(cross(Vo,r1))
hughes_term1_area2 = dot(N2,Vo)*(cross(Vo,r2))

hughes_term2_area1 = ((norm(r1)^2)*eye(3)-r1*r1')*dot(N1,Vo)*w
hughes_term2_area2 = ((norm(r2)^2)*eye(3)-r2*r2')*dot(N2,Vo)*w


hughes_term3_area1 = Smtrx(Vo)*(r1*N1'*(Smtrx(r1)))*w
hughes_term3_area2 = Smtrx(Vo)*(r2*N2'*(Smtrx(r2)))*w

wertz = rho*V^2*(wertz_term1_area1*A1+wertz_term1_area2*A2)...
      + rho*V*(wertz_term2_area1*A1+wertz_term2_area2*A2)...
      + rho*V*(wertz_term3_area1*A1+wertz_term3_area2*A2)

hughes = rho*V^2*(hughes_term1_area1*A1+hughes_term1_area2*A2)...
       - rho*V*(hughes_term2_area1*A1+hughes_term2_area2*A2)...
       - rho*V*(hughes_term3_area1*A1+hughes_term3_area2*A2)

m  = 12;  % [kg] Satellite Mass

jx = 1;   % X-axis Length
jy = 4;   % Y-axis Length
jz = 1;  % Z-axis Length

Jx = (m/12)*(jy^2+jz^2); % X-axis inertia
Jy = (m/12)*(jx^2+jz^2); % Y-axis inertia
Jz = (m/12)*(jx^2+jy^2); % Z-axis inertia
J  = diag([Jx Jy Jz])


