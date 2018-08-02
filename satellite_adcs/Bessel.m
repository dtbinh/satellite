function [J] = Bessel(sig_SS,FOV)
%% This function creates the spacial noise of the Sun Sensor
r     = linspace(eps,FOV);
theta = linspace(0,2*pi)';
n     = length(r);
J     = zeros(n,n);

for i = 1:n
J(i,:) = besseli(rand(n,1),r*r(i))*cos(theta)*sig_SS;
end
J(:,1) = J(:,1)./norm(J(:,1));
J(1,2:n) = J(1,2:n)./norm(J(1,2:n));
return