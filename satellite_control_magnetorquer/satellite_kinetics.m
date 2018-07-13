function output = satellite_kinetics(input)
tau     = input(1:3);       % angular velocity of body ref to inertia in body frame
w_B_IB  = input(4:6);      % Sensor Ax, Ay

global I
%I*w_dot_B_IB = tau - cross(w_B_IB,I*w_B_IB)

w_dot_B_IB = (I)^(-1)*(tau - cross(w_B_IB,I*w_B_IB));


output = w_dot_B_IB;