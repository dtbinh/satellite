close all
clear all
clc

format long
testx = [12.000  -51.000    4.000;
            6.000  167.000  -68.000;
           -4.000   24.000  -41.000;
           -1.000    1.000    0.000;
            2.000    0.000    3.000;
            -2.000    1.000    3.000];
[testq,testr] = qr(testx,0)

% ------------------------------------------------------------------------
% Load data
% file = 'DORBIT_rwa_fm1.dat';  num  = 2;  % File Name
file = 'NARSS_rwa_fm3_short_s.dat';  num  = 5;  % File Name
                    % Number of Variable
filedata = poly_generation_load(file,num); % Load File Variable

data = filedata(1:end,:); 
x = data(:,4)*rpm2rps;
y = data(:,5)/1000;
% ------------------------------------------------------------------------
% Obtain Coefficients
poly_degree = 4;
poly_coeff= polyfit(x,y,poly_degree);

% ------------------------------------------------------------------------
% Fit with Coefficients
spd = 500:10:5000;
spd = spd.*rpm2rps;
poly_fit = polyval(poly_coeff,spd);

for i = 1:1:length(spd)
    trq(i) = poly_coeff(1)*spd(i)^2 + poly_coeff(2)*spd(i) + poly_coeff(3);
end
% ------------------------------------------------------------------------
% Report
fprintf('Determined Polynomial for Fit Calculation\n');
fprintf('Polynomial: %24.21f |  %24.21f |  %24.21f  \n',poly_coeff);

% ------------------------------------------------------------------------
% PLOT
fig = figure;
screensize = get(0,'ScreenSize');
set(fig,'Position',[screensize(3)*0.25 screensize(4)*0.5 screensize(4)*0.8 screensize(4)*0.8]);
plot(x,y,'x')
hold on; grid on;
plot(spd,poly_fit,'r');
plot(spd,trq,'k--');
title('NARSS RWA FM6');
xlabel('Speed [rad/s]');
ylabel('Torque [Nm]');





