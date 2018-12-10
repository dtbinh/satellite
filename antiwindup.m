%------------------------------------------------------------------------
% Anti-Windup Example
%------------------------------------------------------------------------
% This example explains the anti windup in a PID controller of a car moving
% up a slope. Saturation occurs and a comparis with a PID controller with
% antiwindup is used to show the effect
%------------------------------------------------------------------------
set(0,'DefaultAxesFontSize', 12, 'DefaultAxesFontWeight','demi')
set(0,'DefaultTextFontSize', 12, 'DefaultTextFontWeight','demi')
set(0,'DefaultAxesFontName','arial')
set(0,'DefaultAxesFontSize',12)
set(0,'DefaultTextFontName','arial')
clear all
close all
clc


% % Parameters for defining the system dynamics
theta = 0;
gear  = 5;
alpha = [40, 25, 16, 12, 10]; % gear ratios
Tm    = 190; % engine torque constant, Nm
wm    = 420; % peak torque rate, rad/sec
beta  = 0.4;

Cr = 0.01;   % coefficient of rolling friction
rho = 1.2;   % density of air, kg/m^3
A = 2.4;     % car area, m^2
Cd = 0.79/A; % drag coefficient
g = 9.8;     % gravitational constant
m = 1500;    % mass
v = 25;

% % Compute the torque produced by the engine, Tm

omega = alpha(gear) * v;  % engine speed
torque = Tm * ( 1 - beta * (omega/wm - 1)^2 );
F = alpha(gear) * torque;

% % Compute the external forces on the vehicle
Fr   = m * g * Cr; % Rolling friction
Fa   = 0.5 * rho * Cd * A * v^2; % Aerodynamic drag
Fg   = m * g * sin(theta); % Road slope force
Fd   = Fr+Fa+Fg; % total deceleration
ubar = (Fa+Fr)/(F);
vbar = v;
% 
dTdv = Tm*(-2*beta*(alpha(gear)*vbar/wm-1)*(alpha(gear)/wm));
Adyn = (alpha(gear)*dTdv*ubar-rho*Cd*A*vbar)/m;
Bdyn = F/m;
Hillangle = 4; % non−saturating angle
sim('cruise_control');
% figure(4);
% subplot(211);
% plot(Car1(:,5),Car1(:,[2]),'LineWidth',2);xlabel('Time');ylabel('Speed')
% title(['Hill response, angle=',num2str(Hillangle)])
% subplot(212);
% plot(Car1(:,5),Car1(:,[4]),'LineWidth',2);xlabel('Time');ylabel('Throttle')
% figure(1);
% subplot(211);
% plot(Car1(:,5),Car1(:,[1 2]),'LineWidth',2);xlabel('Time');ylabel('Speed')
% legend('NL','Lin','Location','SouthEast');
% title(['Hill response, angle=',num2str(Hillangle)])
% subplot(212);
% plot(Car1(:,5),Car1(:,[3 4]),'LineWidth',2);xlabel('Time');ylabel('Throttle')
% Antiwindup gain=0; % us esame code with and without AWup
% Hillangle=4.85;
% sim('cruise control awup')
% figure(2);
% subplot(211);
% plot(Car2(:,5),Car2(:,[1 2]),'LineWidth',2);xlabel('Time');ylabel('Speed')
% legend('NL','Lin','Location','SouthEast');
% title(['Hill response, angle=',num2str(Hillangle)])
% subplot(212);
% plot(Car2(:,5),Car2(:,[3 4]),'LineWidth',2);xlabel('Time');ylabel('Throttle')
% Car2 no AW=Car2;
% 
% Antiwindup gain=5;
% sim('cruise control awup')
% figure(3);
% 
% subplot(211);
% plot(Car2(:,5),Car2(:,[1 2]),'LineWidth',2);xlabel('Time');ylabel('Speed')
% legend('NL','Lin','Location','SouthEast');
% title(['Hill response, angle=',num2str(Hillangle),' With Anti−windup gain=',num2str(Antiwindup gain)])
% subplot(212);
% plot(Car2(:,5),Car2(:,[3 4]),'LineWidth',2);xlabel('Time');ylabel('Throttle')
% 
% figure(5);
% subplot(211);
% plot(Car2 no AW(:,5),Car2 no AW(:,[6 7]),Car2 no AW(:,5),10*Car2 no AW(:,[8]),'LineWidth',2);xlabel('Time');yla
% legend('sat(u)','u','10*e {vel}','Location','NorthEast');
% axis([4 20 −0.5 5])
% title(['Hill response no AW'])
% subplot(212);
% plot(Car2(:,5),Car2(:,[6 7]),Car2(:,5),10*Car2(:,[8]),'LineWidth',2);xlabel('Time');ylabel('Windup error')
% legend('sat(u)','u','10*e {vel}','Location','NorthEast');
% title(['Hill response with AW'])
% axis([4 20 −0.5 5])
% 
% print −dpng −r300 −f1 AW1.png
% print −dpng −r300 −f2 AW2.png
% print −dpng −r300 −f3 AW3.png
% print −dpng −r300 −f4 AW4.png
% print −dpng −r300 −f5 AW5.png