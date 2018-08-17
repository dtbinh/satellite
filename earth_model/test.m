close all
clear all
clc


X_i = [1;0;0];
Y_i = [0;1;0];
Z_i = [0;0;1];


r_s = [1.1;0;0]; % position vector of satellite in inertial
v_s = [0;0;1];   % velocity vector of satellite in inertial

r_t = vnorm([1;-0.4;0]);

[R_i_r,r1,r2,r3] = triad2dcm(r_t-r_s, v_s);

R_r_i = R_i_r';

los_vec = [0;1;0];  % LOS of payload in body frame
vel_vec = [-1;0;0];  % velocity vector in orbital frame

R_b_r = triad2dcm(los_vec,vel_vec);

R_b_i = R_b_r*R_r_i;
R_i_b = R_b_i';


%% SIMULATION
close all 
createSimulation([0 0 1 1],3)

% Satellite Position
R_sat = plotposition(r_s,'b','s','sat');

% Target Position
R_tgt = plotposition(r_t,'k','s','tgt');

% target vector
X_ref = plotvector(r1,r_s,'r','x_r_e_f',0.5);
Y_ref = plotvector(r2,r_s,'r','y_r_e_f',0.5);
Z_ref = plotvector(r3,r_s,'r','z_r_e_f',0.5);

X_tgt = plotvector(R_i_b*[1;0;0],r_s,'b','x_t_g_t',0.75);
Y_tgt = plotvector(R_i_b*[0;1;0],r_s,'b','y_t_g_t',0.75);
Z_tgt = plotvector(R_i_b*[0;0;1],r_s,'b','z_t_g_t',0.75);

