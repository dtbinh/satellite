function x_dot = satellite_dynamics(input)
q      = input(1:4);       % quaternions of body ref to inertia in body frame
w_bi   = input(5:7);       % angular velocity of body ref to inertia in body frame
T_ext  = input(8:10);      % Sensor Ax, Ay

global I_sc

%% QUATERNION MATRIX PROPAGATION
omega = [0         w_bi(3)    -w_bi(2)  w_bi(1);
         -w_bi(3)  0           w_bi(1)  w_bi(2);
          w_bi(2)  -w_bi(1)    0        w_bi(3);
         -w_bi(1)  -w_bi(2)   -w_bi(3)      0   ];

q_dot = 0.5*omega*q;

%% DYNAMICS PROPAGATION IN BODY FRAME
%  Dynamics of a Rigid Body Spacecraft is shown below the where the torques 
%  and rates are about body frame axes and the cross-product terms create 
%  crosscoupling between the axes:
%    I_sc*w_dot+ w_b x (I*w_b)=T_ext;

w_dot = I_sc^(-1)*(T_ext - cross(w_bi,I_sc*w_bi));


x_dot = [q_dot; w_dot];