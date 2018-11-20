close all
clear all
clc

format long

%% SATELLITE DATA
satellite = 'opssat';
screensize   = get(0,'ScreenSize');
screenwidth  = screensize(3);
screenheight = screensize(4);
screennumber = 1;
fig = figure;
set(fig,'Position',[screenwidth*(screennumber+0) 0 screenwidth*0.35 screenheight]);
fprintf(strcat('\n                [',upper(satellite),']\n'));
for i=1:1:3
    fprintf('\n=================Gyro %d===================\n',i);
    %% LOAD DATA
    gyro      = i-1;
    file   = strcat(satellite,'_gyro',num2str(gyro),'.dat');
    fileID = fopen(file);
    wdata  = textscan(fileID,'%19c %f %f %f | %4c %f %11c %f %6c');
    fclose(fileID);

    %% INPUT PARAMETER
    w0     = wdata{2};
    w1     = wdata{3};
    w2     = wdata{4};
    w_abs  = wdata{6};
    t      = wdata{8};
    t      = (t-t(1))/1000;
    
    %% STANDARD DEVATION
    fprintf('Mean   :%8.5f | %8.5f | %8.5f [dps]\n',mean(w0),mean(w1),mean(w2));
    fprintf('1-Std  :%8.5f | %8.5f | %8.5f [dps]\n',std(w0),std(w1),std(w2));
    fprintf('3-Std  :%8.5f | %8.5f | %8.5f [dps]\n',std(w0)*3,std(w1)*3,std(w2)*3);
    
    %% BIAS STABILITY
    
    tend = 0;
    dnum = 5;
    div = ceil(length(t)/dnum);
    
    
        for k=1:1:dnum
            fprintf('Bias[%d]:',k);

            sum = 0;
            tstart = tend+1;
            tend = min(div*k,length(t));
                for j=tstart:1:tend
                    sum = sum + w0(j);
                end
            bias(k) = sum/div;
            fprintf('%8.5f | ',bias(k));
            
            sum = 0;
                for j=tstart:1:tend
                    sum = sum + w1(j);
                end
            bias(k) = sum/div;
            fprintf('%8.5f | ',bias(k));
            
            sum = 0;
                for j=tstart:1:tend
                    sum = sum + w2(j);
                end
            bias(k) = sum/div;
            fprintf('%8.5f [dps]',bias(k));
            
            
        fprintf('\n');
        end
    
    
    %% PLOT
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


