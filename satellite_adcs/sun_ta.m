function output = sun_ta(input)
R = input(1:3);
V = input(4:6);
S_eci = input(7:9);

H = cross(R,V);                    % [m^2/s] Angular Momentum Vector Z axis of Perifocal Frame
S_X = cross(S_eci,H);
S_Y = cross(H,S_X);

S_Y = S_Y/norm(S_Y);



ta = acos(dot(S_Y,R)/norm(S_Y)/norm(R));
if acos(dot(-S_X,R)/norm(-S_X)/norm(R))>pi/2
    ta = 2*pi-ta;
end
output = ta;
end