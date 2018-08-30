close all
clear all
clc

%% VECTORS MEASUREMENT
v1b = [0.8273; 0.5541; -0.0920];
v2b = [-0.8285; 0.5522; -0.0955];

%% VECTORS INERTIAL
v1i = [-0.1517; -0.9669; 0.2050];
v2i = [-0.8393; 0.4494; -0.3044];

%% TRIAD ALGORITHM VECTORS

t1b = v1b
t2b = cross(v1b, v2b)/norm(cross(v1b, v2b))
t3b = cross(t1b,t2b)

t1i = v1i
t2i = cross(v1i,v2i)/norm(cross(v1i, v2i))
t3i = cross(t1i,t2i)

%% ESTIMATED ROTATION MATRIX

Rbt = [t1b t2b t3b]
Rti = [t1i t2i t3i]'

Rbi = Rbt*Rti

