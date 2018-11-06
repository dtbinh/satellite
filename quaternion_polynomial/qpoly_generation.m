close all
clear all
clc

format long

%% LOAD DATA
file    = 'qdata4.dat';
qpoly_generation_load

%% INPUT PARAMETER
dt_step   = 0.1;                                    % [sec] Time for each simulation step
row_start = find(strcmp(utc_time,'11:18:00.000'));  % [ - ] Start Row
row_end   = find(strcmp(utc_time,'11:20:20.000'));  % [ - ] End Row
row_length = row_end-row_start;                     % [ - ] Row Length
division  = 10;                                     % [ - ] Number of Division
n_step    = row_length/division;                    % [ - ] step - each step is 0.1 sec
t_step    = n_step*0.1;                             % [sec] time block for each quaternion

qtgt_acq(:,1) = [qtgt0(row_start);qtgt1(row_start);qtgt2(row_start);qtgt3(row_start)];
ttgt_acq(1,1) = 0;                    % [sec] Initial start 
i             = 2;                    % Initialise i = 2 since i = 1 is determined

fprintf('(%2d) t: %5.2f q:%10.6f |%10.6f | %10.6f | %10.6f |\n',1,ttgt_acq(1,1),qtgt_acq(:,1));
for n = row_start+n_step:n_step:row_start+row_length
    
    ttgt_acq(1,i) = ttgt_acq(1,i-1) + t_step;
    qtgt_acq(:,i) = [qtgt0(n);qtgt1(n);qtgt2(n);qtgt3(n)];
    
    fprintf('(%2d) t: %5.2f q:%10.6f |%10.6f | %10.6f | %10.6f |\n',i,ttgt_acq(1,i),qtgt_acq(:,i));
    i = i+1;
end

%% qPoly 
tdur_acq    = 0:0.1:ttgt_acq(1,end);

qpoly_acq_degree = 6;
qpoly_coeff_acq(1,:) = polyfit(ttgt_acq,qtgt_acq(1,:),qpoly_acq_degree);
qpoly_coeff_acq(2,:) = polyfit(ttgt_acq,qtgt_acq(2,:),qpoly_acq_degree);
qpoly_coeff_acq(3,:) = polyfit(ttgt_acq,qtgt_acq(3,:),qpoly_acq_degree);
qpoly_coeff_acq(4,:) = polyfit(ttgt_acq,qtgt_acq(4,:),qpoly_acq_degree);


qfit_acq(1,:) = polyval(qpoly_coeff_acq(1,:),tdur_acq);
qfit_acq(2,:) = polyval(qpoly_coeff_acq(2,:),tdur_acq);
qfit_acq(3,:) = polyval(qpoly_coeff_acq(3,:),tdur_acq);
qfit_acq(4,:) = polyval(qpoly_coeff_acq(4,:),tdur_acq);

for i=1:1:length(qfit_acq)
    qfit_acq_norm(:,i) = qnorm(qfit_acq(:,i));
end

% print
fprintf('Determined Quaternion for Fit Calculation\n');
fprintf('Quaternion Acquistion Start:         %10.7f |  %10.7f |  %10.7f |  %10.7f | %10.7f |\n',qtgt_acq(:,1),norm(qtgt_acq(:,1)))
fprintf('Quaternion Acquistion End:           %10.7f |  %10.7f |  %10.7f |  %10.7f | %10.7f |\n',qtgt_acq(:,end),norm(qtgt_acq(:,end)))
fprintf('\n');
fprintf('Calculated Quaternion Polynomial Fit\n');
fprintf('Quaternion Fit Acquistion Start:     %10.7f |  %10.7f |  %10.7f |  %10.7f | %10.7f |\n',qfit_acq(:,1),norm(qfit_acq(:,1)))
fprintf('Quaternion Fit Acquistion End:       %10.7f |  %10.7f |  %10.7f |  %10.7f | %10.7f |\n',qfit_acq(:,end),norm(qfit_acq(:,end)))
fprintf('\n');
fprintf('Acquistion\n');
fprintf('QPoly q[0]: %30.25f |  %30.25f |  %30.25f |  %30.25f | %30.25f | %30.25f | %30.25f |\n',qpoly_coeff_acq(1,:));
fprintf('QPoly q[1]: %30.25f |  %30.25f |  %30.25f |  %30.25f | %30.25f | %30.25f | %30.25f |\n',qpoly_coeff_acq(2,:));
fprintf('QPoly q[2]: %30.25f |  %30.25f |  %30.25f |  %30.25f | %30.25f | %30.25f | %30.25f |\n',qpoly_coeff_acq(3,:));
fprintf('QPoly q[3]: %30.25f |  %30.25f |  %30.25f |  %30.25f | %30.25f | %30.25f | %30.25f |\n',qpoly_coeff_acq(4,:));

% Printf for Satsim
fprintf('\n11:15:15.000\n')
for j=0:1:3
    
    fprintf('\n');
    for i=0:1:qpoly_acq_degree

    fprintf('%30.25f | ',qpoly_coeff_acq(j+1,qpoly_acq_degree+1-i));

    end

end
 fprintf('\n\n');
  
%% PLOT
figure
ttgt_act = 0.0:0.1:ttgt_acq(1,end);
plot(ttgt_act,qtgt0(row_start:row_end),'--')
hold on; grid on;
plot(ttgt_act,qtgt1(row_start:row_end),'--')
plot(ttgt_act,qtgt2(row_start:row_end),'--')
plot(ttgt_act,qtgt3(row_start:row_end),'--')

plot(ttgt_acq,qtgt_acq,'x')
hold on; grid on;
plot(tdur_acq,qfit_acq(1,:),'b')
plot(tdur_acq,qfit_acq(2,:),'b')
plot(tdur_acq,qfit_acq(3,:),'b')
plot(tdur_acq,qfit_acq(4,:),'b')

plot(tdur_acq,qfit_acq_norm(1,:),'r')
plot(tdur_acq,qfit_acq_norm(2,:),'r')
plot(tdur_acq,qfit_acq_norm(3,:),'r')
plot(tdur_acq,qfit_acq_norm(4,:),'r')

axis([-inf inf -1 1])




