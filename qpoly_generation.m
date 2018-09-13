close all
clear all
clc

format long
fileID = fopen('qdata.dat');
qheader1 = textscan(fileID,'%4c | %31c| %22c | %24c | %18c |',1);
qheader2 = textscan(fileID,'%s | %s | %9c | %4c | %4c | %4c | %4c |%8c |%8c |%8c |%5c |%5c |%5c |%5c |%5c |%5c |%9c |%5c |%5c |%5c |%5c |%5c |%5c |%10c |%4c |%4c |%4c |%4c |%5c |%5c |%5c |%9c |%9c |%9c |%c|%c|%11c|%9c|%9c|%11c|',1);
qdata    = textscan(fileID,'%d | %s %s |%d | %f | %f | %f | %f | %f | %f | %f | %f | %f | %f | %f | %f | %f | %f | %f | %f | %f | %f | %f | %f | %f | %f | %f | %f | %f | %f | %f | %f | %f | %f | %f | %d | %d | %s | %f | %f | %f | ');
fclose(fileID);

Name = qdata{1};
utc_data  = qdata{2};
utc_time  = qdata{3};
dt   = qdata{4};
q0   = qdata{5};
q1   = qdata{6};
q2   = qdata{7};
q3   = qdata{8};
ra   = qdata{9};
de   = qdata{10};
az   = qdata{11};

hb0   = qdata{12};
hb1   = qdata{13};
hb2   = qdata{14};
hi0   = qdata{15};
hi1   = qdata{16};
hi2   = qdata{17};

habs  = qdata{18};
wb0  = qdata{19};
wb1  = qdata{20};
wb2  = qdata{21};
wi0  = qdata{22};
wi1  = qdata{23};
wi2  = qdata{24};
wabs  = qdata{25};

qtgt0  = qdata{26};
qtgt1  = qdata{27};
qtgt2  = qdata{28};
qtgt3  = qdata{29};

wtgtb0  = qdata{30};
wtgtb1  = qdata{31};
wtgtb2  = qdata{32};
wtgti0  = qdata{33};
wtgti1  = qdata{34};
wtgti2  = qdata{35};

V = qdata{36};
T = qdata{37};

NADIR = qdata{37};
STact_sun = qdata{38};
STact_earth = qdata{39};

% find q target for manuveure
row = find(strcmp(utc_time,'06:12:00.000'));
qtgt_mnvr(:,1) = [q0(row);q1(row);q2(row);q3(row)];
ttgt_mnvr(1,1)    = 0.0;

row = find(strcmp(utc_time,'06:14:00.000'));
qtgt_mnvr(:,2) = [qtgt0(row);qtgt1(row);qtgt2(row);qtgt3(row)];
ttgt_mnvr(1,2)    = 120.0;

row = find(strcmp(utc_time,'06:15:30.000'));
qtgt_mnvr(:,3) = [qtgt0(row);qtgt1(row);qtgt2(row);qtgt3(row)];
ttgt_mnvr(1,3)    = 210.0;

row = find(strcmp(utc_time,'06:17:00.000'));
qtgt_mnvr(:,4) = [qtgt0(row);qtgt1(row);qtgt2(row);qtgt3(row)];
ttgt_mnvr(1,4)    = 300.0;

row = find(strcmp(utc_time,'06:17:30.000'));
qtgt_mnvr(:,5) = [qtgt0(row);qtgt1(row);qtgt2(row);qtgt3(row)];
ttgt_mnvr(1,5)    = 330.0;



% find q target for imaging
row = find(strcmp(utc_time,'06:17:30.000'));
qtgt_acq(:,1) = [qtgt0(row);qtgt1(row);qtgt2(row);qtgt3(row)];
ttgt_acq(1,1) = 0;    % [sec] Initial start 
n_step        = (4801-3301)/10;       % [-]   step - each step is 0.1 sec
t_step        = n_step*0.1;   % [sec] time step
i = 2;

for n = row+n_step:n_step:length(utc_time)
    
    ttgt_acq(1,i) = ttgt_acq(1,i-1) + t_step;
    
    qtgt_acq(:,i) = [qtgt0(n);qtgt1(n);qtgt2(n);qtgt3(n)];
    
    i = i+1;
end


% qPoly Manueveurve
tdur_mnvr    = 0.0:0.1:330.0;

qpoly_mnvr_degree = 3;
qpoly_coeff_mnvr(1,:) = polyfit(ttgt_mnvr,qtgt_mnvr(1,:),qpoly_mnvr_degree);
qpoly_coeff_mnvr(2,:) = polyfit(ttgt_mnvr,qtgt_mnvr(2,:),qpoly_mnvr_degree);
qpoly_coeff_mnvr(3,:) = polyfit(ttgt_mnvr,qtgt_mnvr(3,:),qpoly_mnvr_degree);
qpoly_coeff_mnvr(4,:) = polyfit(ttgt_mnvr,qtgt_mnvr(4,:),qpoly_mnvr_degree);

qfit_mnvr(1,:) = polyval(qpoly_coeff_mnvr(1,:),tdur_mnvr);
qfit_mnvr(2,:) = polyval(qpoly_coeff_mnvr(2,:),tdur_mnvr);
qfit_mnvr(3,:) = polyval(qpoly_coeff_mnvr(3,:),tdur_mnvr);
qfit_mnvr(4,:) = polyval(qpoly_coeff_mnvr(4,:),tdur_mnvr);

% qPoly Imaging
tdur_acq    = 0:0.1:150.0;

qpoly_acq_degree = 6;
qpoly_coeff_acq(1,:) = polyfit(ttgt_acq,qtgt_acq(1,:),qpoly_acq_degree);
qpoly_coeff_acq(2,:) = polyfit(ttgt_acq,qtgt_acq(2,:),qpoly_acq_degree);
qpoly_coeff_acq(3,:) = polyfit(ttgt_acq,qtgt_acq(3,:),qpoly_acq_degree);
qpoly_coeff_acq(4,:) = polyfit(ttgt_acq,qtgt_acq(4,:),qpoly_acq_degree);

qfit_acq(1,:) = polyval(qpoly_coeff_acq(1,:),tdur_acq);
qfit_acq(2,:) = polyval(qpoly_coeff_acq(2,:),tdur_acq);
qfit_acq(3,:) = polyval(qpoly_coeff_acq(3,:),tdur_acq);
qfit_acq(4,:) = polyval(qpoly_coeff_acq(4,:),tdur_acq);

% print
fprintf('Determined Quaternion for Fit Calculation\n');
fprintf('Quaternion Manuever Start:           %10.7f |  %10.7f |  %10.7f |  %10.7f | %10.7f |\n',qtgt_mnvr(:,1),norm(qtgt_mnvr(:,1)))
fprintf('Quaternion Manuever End:             %10.7f |  %10.7f |  %10.7f |  %10.7f | %10.7f |\n',qtgt_mnvr(:,end),norm(qtgt_mnvr(:,end)))
fprintf('Quaternion Acquistion Start:         %10.7f |  %10.7f |  %10.7f |  %10.7f | %10.7f |\n',qtgt_acq(:,1),norm(qtgt_acq(:,1)))
fprintf('Quaternion Acquistion End:           %10.7f |  %10.7f |  %10.7f |  %10.7f | %10.7f |\n',qtgt_acq(:,end),norm(qtgt_acq(:,end)))
fprintf('\n');
fprintf('Calculated Quaternion Polynomial Fit\n');
fprintf('Quaternion Fit Manuever Start:       %10.7f |  %10.7f |  %10.7f |  %10.7f | %10.7f |\n',qfit_mnvr(:,1),norm(qfit_mnvr(:,1)))
fprintf('Quaternion Fit Manuever End:         %10.7f |  %10.7f |  %10.7f |  %10.7f | %10.7f |\n',qfit_mnvr(:,end),norm(qfit_mnvr(:,end)))
fprintf('Quaternion Fit Acquistion Start:     %10.7f |  %10.7f |  %10.7f |  %10.7f | %10.7f |\n',qfit_acq(:,1),norm(qfit_acq(:,1)))
fprintf('Quaternion Fit Acquistion End:       %10.7f |  %10.7f |  %10.7f |  %10.7f | %10.7f |\n',qfit_acq(:,end),norm(qfit_acq(:,end)))
fprintf('\n');
fprintf('Manuever\n');
fprintf('QPoly q[0]: %30.25f |  %30.25f |  %30.25f |  %30.25f | %30.25f |  %30.25f |\n',qpoly_coeff_mnvr(1,:));
fprintf('QPoly q[1]: %30.25f |  %30.25f |  %30.25f |  %30.25f | %30.25f |  %30.25f |\n',qpoly_coeff_mnvr(2,:));
fprintf('QPoly q[2]: %30.25f |  %30.25f |  %30.25f |  %30.25f | %30.25f |  %30.25f |\n',qpoly_coeff_mnvr(3,:));
fprintf('QPoly q[3]: %30.25f |  %30.25f |  %30.25f |  %30.25f | %30.25f |  %30.25f |\n',qpoly_coeff_mnvr(4,:));
fprintf('\n');
fprintf('Acquistion\n');
fprintf('QPoly q[0]: %30.25f |  %30.25f |  %30.25f |  %30.25f | %30.25f | %30.25f | %30.25f |\n',qpoly_coeff_acq(1,:));
fprintf('QPoly q[1]: %30.25f |  %30.25f |  %30.25f |  %30.25f | %30.25f | %30.25f | %30.25f |\n',qpoly_coeff_acq(2,:));
fprintf('QPoly q[2]: %30.25f |  %30.25f |  %30.25f |  %30.25f | %30.25f | %30.25f | %30.25f |\n',qpoly_coeff_acq(3,:));
fprintf('QPoly q[3]: %30.25f |  %30.25f |  %30.25f |  %30.25f | %30.25f | %30.25f | %30.25f |\n',qpoly_coeff_acq(4,:));

% Printf for Rusty Satsim
for j=0:1:3
    
    fprintf('\n');
    for i=0:1:qpoly_mnvr_degree

    fprintf('multi_qpoly_coeff[%d] =  %30.25f\n',j*(qpoly_mnvr_degree+1)+i,qpoly_coeff_mnvr(j+1,qpoly_mnvr_degree+1-i));

    end
end

for j=0:1:3
    
    fprintf('\n');
    for i=0:1:qpoly_acq_degree

    fprintf('multi_qpoly_coeff[%d] =  %30.25f\n',j*(qpoly_acq_degree+1)+i+28,qpoly_coeff_acq(j+1,qpoly_acq_degree+1-i));

    end
end
fprintf('\n\n');

% Printf for Satsim
fprintf('2020 | 6 | 1 | 6 | 12 | 0 | 0 | \n')
for j=0:1:3
    
    fprintf('\n');
    for i=0:1:qpoly_mnvr_degree

    fprintf('%30.25f | ',qpoly_coeff_mnvr(j+1,qpoly_mnvr_degree+1-i));

    end

end
fprintf('\n\n');

fprintf('2020 | 6 | 1 | 6 | 17 | 30 | 0 | \n')
for j=0:1:3
    
    fprintf('\n');
    for i=0:1:qpoly_acq_degree

    fprintf('%30.25f | ',qpoly_coeff_acq(j+1,qpoly_acq_degree+1-i));

    end

end
 fprintf('\n');
  fprintf('\n');
%% plot
figure
ttgt_act = 0.0:0.1:480;
plot(ttgt_act,qtgt0,'--')
hold on; grid on;
plot(ttgt_act,qtgt1,'--')
plot(ttgt_act,qtgt2,'--')
plot(ttgt_act,qtgt3,'--')

plot(ttgt_mnvr,qtgt_mnvr,'o')

plot(tdur_mnvr,qfit_mnvr(1,:))
plot(tdur_mnvr,qfit_mnvr(2,:))
plot(tdur_mnvr,qfit_mnvr(3,:))
plot(tdur_mnvr,qfit_mnvr(4,:))

plot(ttgt_acq+330,qtgt_acq,'x')
hold on; grid on;
plot(tdur_acq+330,qfit_acq(1,:))
plot(tdur_acq+330,qfit_acq(2,:))
plot(tdur_acq+330,qfit_acq(3,:))
plot(tdur_acq+330,qfit_acq(4,:))



axis([-inf inf -1 1])


