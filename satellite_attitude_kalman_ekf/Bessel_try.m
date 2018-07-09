close all
clear
clc
%% This function creates the spacial noise of the Sun Sensor

FOV   = 0.7; % [-] Field of View
sigSS = 0.1; % [T] Standard Deviation
eps
r     = linspace(eps,FOV); % Array [1x100] from eps to FOV in 100 spacing
theta = linspace(0,2*pi)'; % Array [100x1] from eps to FOV in 100 spacing
n     = length(r);         % Length of r
J     = zeros(n,n);        % Array [100x100] from eps to FOV in 100 spacing

%%

for i = 1:1:n
    R(i,:) = r(i)*r;
    
J(i,:) = besseli(0.01*rand(1,n),R(i,:));%;
end
J(:,1) = J(:,1)./norm(J(:,1));
J(1,2:n) = J(1,2:n)./norm(J(1,2:n));
figure
mesh(R)
figure
mesh(J)
