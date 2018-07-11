% Generate sun sensor response graph from P6 notes.

close all
clear variables

% generate output measurements for a single axis (i.e. the measured alpha
% or beta angles) given the true input angle (to the sensor) 
j=1;
for i=-90:90
	true_angle(j) = i*pi/180;
    meas_angle(j) = SunSensorResponse(true_angle(j));
	j = j+1;
end

% plot response characteristic
figure
plot(true_angle*180/pi,meas_angle*180/pi),grid on, zoom on
xlabel('\alpha [deg]')
ylabel('\alpha_{meas} [deg]')
title('Sun Sensor Response Output Vs. Input Angle')
ax=axis;
axis([-90 90 ax(3:4)])

hold on
plot(7*[1 1], ax(3:4),'r--')
plot(-7*[1 1], ax(3:4),'r--')
plot(83*[1 1], ax(3:4),'r--')
plot(-83*[1 1], ax(3:4),'r--')
