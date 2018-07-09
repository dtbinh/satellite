%% PLOT CONTROLLER RATE OF SPACECRAFT
figure
subplot(3,1,1)
plot(t,w_bi(1,:)*r2d);
grid on, zoom on, hold all
plot(t,w_bi_meas(1,:)*r2d)
xlabel('time [sec]')
ylabel('w_{bi} [deg/sec]')
title('Inertial Body Rate X in Body Frame')

subplot(3,1,2)
plot(t,w_bi(2,:)*r2d);
grid on, zoom on, hold all
plot(t,w_bi_meas(2,:)*r2d)
xlabel('time [sec]')
ylabel('w_{bi} [deg/sec]')
title('Inertial Body Rate Y in Body Frame')

subplot(3,1,3)
plot(t,w_bi(3,:)*r2d);
grid on, zoom on, hold all
plot(t,w_bi_meas(3,:)*r2d)
xlabel('time [sec]')
ylabel('w_{bi} [deg/sec]')
title('Inertial Body Rate Z in Body Frame')


%% PLOT TORQUE DEMAND AND TRUE TORQUE
figure
plot(t,T_dem), hold on, grid on, zoom on
plot(t,T_rcs)

plot([t(1) t(end)],+Tdb(1)*[1 1],'--',[t(1) t(end)],+Tdb(2)*[1 1],'--',[t(1) t(end)],+Tdb(3)*[1 1],'--')
plot([t(1) t(end)],-Tdb(1)*[1 1],'--',[t(1) t(end)],-Tdb(2)*[1 1],'--',[t(1) t(end)],-Tdb(3)*[1 1],'--')
xlabel('time [sec]')
ylabel('RCS torque demand & truth [Nm]')
title('Controller Torque demand and Actual RCS Torque')
legend('X','Y','Z')

%% PLOT CONTROLLER ATTITUDE ERROR 
figure
h = plot(t,alpha_error*r2d,t,beta_error*r2d);
set(h(1),'LineWidth', 2)
set(h(2),'LineWidth', 2)
hold on, grid on, zoom on
plot([t(1) t(end)], +r2d*attitude_deadband_x*[1 1],'b--', [t(1) t(end)], -r2d*attitude_deadband_x*[1 1],'b--' )
plot([t(1) t(end)], +r2d*attitude_deadband_y*[1 1],'g--', [t(1) t(end)], -r2d*attitude_deadband_y*[1 1],'g--' )
xlabel('time [sec]')
ylabel('attitude errors [deg]')
title('Attitude Controller Errors for [X Y] Axes')
legend('\alpha','\beta')

%% PLOT CONTROLLER RATE ERRORS X & Y
figure
plot(t,w_bi(1:2,:)*r2d), grid on, zoom on
xlabel('time [sec]')
ylabel('w_{bi} [deg/sec]')
title('Controller Rate Errors for [X Y] Axes')
legend('X','Y')

%% PLOT CONTROLLER RATE ERROR Z
figure
plot(t,wz_error*r2d), hold on, grid on, zoom on
plot([t(1) t(end)], +r2d*rate_deadband_z*[1 1],'r--', [t(1) t(end)], -r2d*rate_deadband_z*[1 1],'r--' )
xlabel('time [sec]')
ylabel('rate error [deg/sec]')
title('Rate Controller Error for [Z] axis with Rate DeadBand')
legend('Z Rate Error','DeadBand');