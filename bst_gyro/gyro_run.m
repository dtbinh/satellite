% -------------------------------------------------------------------------
% GYRO NOISE PLOT
% -------------------------------------------------------------------------
% This program plots the gyro noise and measures the standard deviation of
% the measurements from .dat files. The format of the file is as follow:
% 
% Gyro values [b-fr]:  -0.006  -0.001   0.024 | Abs.   0.025 [dps] @ t=        1307811[msec]
% Gyro values [b-fr]:   0.018   0.004   0.042 | Abs.   0.046 [dps] @ t=        1308912[msec]
% Gyro values [b-fr]:   0.048   0.010   0.011 | Abs.   0.050 [dps] @ t=        1310012[msec]
%
% Author: Rusty Goh
% Date:   17 Dec 2018 
% ------------------------------------------------------------------------
close all
clear all
clc

format long
% ------------------------------------------------------------------------
% SATELLITE DATA
satellite    = 'trisat';
screensize   = get(0,'ScreenSize');
screenwidth  = screensize(3);
screenheight = screensize(4);
screennumber = 0;
fig = figure;
set(fig,'Position',[screenwidth*(screennumber+0) 0 screenwidth*0.35 screenheight]);
fprintf(strcat('\n                [',upper(satellite),']\n'));

for i=1:1:3
    
fprintf('\n=================Gyro %d===================\n',i);
fprintf('        %8c | %8c | %8c |    \n','x','y','z');
% --------------------------------------------------------------------
% LOAD DATA
gyro     = i-1;
file     = strcat(satellite,'_gyro',num2str(gyro),'.dat');
fileID   = fopen(file);
wdata    = textscan(fileID,'%19c %f %f %f | %4c %f %11c %f %6c');
fclose(fileID);

% --------------------------------------------------------------------
% INPUT PARAMETER
w0     = wdata{2};
w1     = wdata{3};
w2     = wdata{4};
w_abs  = wdata{6};
t      = wdata{8};
t      = (t-t(1))/1000;

% --------------------------------------------------------------------
% STANDARD DEVATION
fprintf('Mean   :%8.5f | %8.5f | %8.5f [dps]\n',mean(w0),mean(w1),mean(w2));
fprintf('1-Std  :%8.5f | %8.5f | %8.5f [dps]\n',std(w0),std(w1),std(w2));
fprintf('3-Std  :%8.5f | %8.5f | %8.5f [dps]\n',std(w0)*3,std(w1)*3,std(w2)*3);

% --------------------------------------------------------------------
% BIAS STABILITY
% Calculate the bias by averaging it at different time segments. The whole
% period is divided by dnum. Bias of each segment is calculated.

tend = 0;
dnum = 5;
div = ceil(length(t)/dnum);

for k=1:1:dnum
    fprintf('Bias[%d]:',k);
    
    % X-axis
    sum = 0;
    tstart = tend+1;
    tend = min(div*k,length(t));
    for j=tstart:1:tend
        sum = sum + w0(j);
    end
    bias(k) = sum/div;
    fprintf('%8.5f | ',bias(k));

    % Y-axis
    sum = 0;
    for j=tstart:1:tend
        sum = sum + w1(j);
    end
    bias(k) = sum/div;
    fprintf('%8.5f | ',bias(k));

    % Z-axis
    sum = 0;
    for j=tstart:1:tend
        sum = sum + w2(j);
    end
    bias(k) = sum/div;
    fprintf('%8.5f [dps]',bias(k));


fprintf('\n');
end

% --------------------------------------------------------------------
% PLOT
stddev = 0.1;
subplot(3,1,i)
plot(t,w0,'r');
title(strcat(satellite,' gyro',num2str(gyro)));
grid on; hold on;
% plot(t, stddev*3*ones(size(t,1))+mean(w0),'r--');
% plot(t,-stddev*3*ones(size(t,1))+mean(w0),'r--');

plot(t,w1,'g');
% plot(t, stddev*3*ones(size(t,1))+mean(w1),'g--');
% plot(t,-stddev*3*ones(size(t,1))+mean(w1),'g--');

plot(t,w2,'b');
% plot(t, stddev*3*ones(size(t,1))+mean(w2),'b--');
% plot(t,-stddev*3*ones(size(t,1))+mean(w2),'b--');

xtext = strcat('x-axis std:',num2str(std(w0)));
ytext = strcat('y-axis std:',num2str(std(w1)));
ztext = strcat('z-axis std:',num2str(std(w2)));

legend(xtext,ytext,ztext);
ylabel('Gyro Measurement [dps]');
xlabel('Simulation Time [sec]');

end

    
% print(fig,strcat(satellite,'_gyro'),'-dpdf','-bestfit');
% saveas(fig,strcat(satellite,'_gyro'),'pdf');


