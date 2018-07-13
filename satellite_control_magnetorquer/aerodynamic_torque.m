function tau_a = aerodynamic_torque(input)
R_B_O(:,1) = input(1:3);
R_B_O(:,2) = input(4:6);
R_B_O(:,3) = input(7:9);
w_B_OB     = input(10:12);

global m 
global I
global w_O
global Rc

jx = 0.03;   % X-axis Length
jy = 0.03;   % Y-axis Length
jz = 0.083;  % Z-axis Length

Jx = (m/12)*(jy^2+jz^2); % X-axis inertia
Jy = (m/12)*(jx^2+jz^2); % Y-axis inertia
Jz = (m/12)*(jx^2+jy^2); % Z-axis inertia
J  = diag([Jx Jy Jz]);

x = 0.1;
y = 0.1;
z = 0.2;

% A_xo = x*z; % Outer satellite area, x-panel
% A_yo = y*z; % Outer satellite area, y-panel
% A_zo = x*y; % Outer satellite area, z-panel

A_drag = z*sqrt(x^2+y^2); % Maximum area for calc disturbance drag

vel   = Rc*w_O;   % [m/s] Velocity of Satellite at Orbit
rho_a = 4.89e-13; % [kg/m^3] density of atmosphere

c_p   = [0.02 0.02 0.02]';  % centre of pressure in BODY components
c_p_x = Smtrx(c_p);         % computes the 3x3 vector cross product matrix S=-S' such that rxt = S(r)t is true for all 3x1 vectors r and t
Vr    = R_B_O*[0 -1 0]';     
Vr_x  = Smtrx(Vr);           % computes the 3x3 vector cross product matrix S=-S' such that rxt = S(r)t is true for all 3x1 vectors r and t


tau_a = rho_a*vel*(vel*A_drag*c_p_x*Vr - (I + Vr_x*J)*w_B_OB);

end