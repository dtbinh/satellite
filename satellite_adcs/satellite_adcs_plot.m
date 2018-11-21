close all

screensize   = get(0,'ScreenSize');
screenwidth  = screensize(3);
screenheight = screensize(4);
screennumber = 0;

%% QUATERNION ERROR %%
ymin = -0.005;
ymax = 0.005;
textx = 5;
texty = ymax*0.8;

fig = figure;
set(fig,'Position',[screenwidth*(screennumber+0) 0 screenwidth*0.25 screenheight]);

subplot(4,1,1)
plot(tout,q_B_I_error(1,:),'b+-.')
title('Quaternion Error')
hold on;grid on;
plot(tout,q_B_I_error_ukf(1,:),'r-.')
plot(tout,q_B_I_error_skf(1,:),'g-.')
ylabel('x [-]');
xlabel('time [s]');
axis([-Inf Inf ymin ymax])
text(textx,texty,strcat('MEAN:',num2str(mean(q_B_I_error(1,ceil(end/4):end))),' STD:',num2str(std(q_B_I_error(1,ceil(end/4):end)))));

subplot(4,1,2)
plot(tout,q_B_I_error(2,:),'b+-.')
hold on;grid on;
plot(tout,q_B_I_error_ukf(2,:),'r-.')
plot(tout,q_B_I_error_skf(2,:),'g-.')
ylabel('y [-]');
xlabel('time [s]');
axis([-Inf Inf ymin ymax])
text(textx,texty,strcat('MEAN:',num2str(mean(q_B_I_error(2,ceil(end/4):end))),' STD:',num2str(std(q_B_I_error(2,ceil(end/4):end)))));

subplot(4,1,3)
plot(tout,q_B_I_error(3,:),'b+-.')
hold on;grid on;
plot(tout,q_B_I_error_ukf(3,:),'r-.')
plot(tout,q_B_I_error_skf(3,:),'g-.')
ylabel('z [-]');
xlabel('time [s]');
axis([-Inf Inf ymin ymax])
text(textx,texty,strcat('MEAN:',num2str(mean(q_B_I_error(3,ceil(end/4):end))),' STD:',num2str(std(q_B_I_error(3,ceil(end/4):end)))));

subplot(4,1,4)
plot(tout,q_B_I_error(4,:),'b+-.')
hold on;grid on;
plot(tout,q_B_I_error_ukf(4,:),'r-.')
plot(tout,q_B_I_error_skf(4,:),'g-.')
ylabel('w [-]');
xlabel('time [s]');
axis([-Inf Inf ymin ymax])
text(textx,texty,strcat('MEAN:',num2str(mean(q_B_I_error(4,ceil(end/4):end))),' STD:',num2str(std(q_B_I_error(4,ceil(end/4):end)))));

% QUATERNION TO INERTIAL FRAME 
fig = figure;
set(fig,'Position',[screenwidth*(screennumber+0) 0 screenwidth*0.25 screenheight]);
subplot(4,1,1)
plot(tout,q_B_I(1,:),'k')
title('Quaternion Estimate')
hold on;grid on;
ylabel('x [-]');
xlabel('time [s]');
plot(tout,q_B_I_f(1,:),'b+-.')
plot(tout,q_B_I_ukf(1,:),'r-.')
plot(tout,q_B_I_skf(1,:),'g-.')

subplot(4,1,2)
plot(tout,q_B_I(2,:),'k')
hold on;grid on;
ylabel('y [-]');
xlabel('time [s]');
plot(tout,q_B_I_f(2,:),'b+-.')
plot(tout,q_B_I_ukf(2,:),'r-.')
plot(tout,q_B_I_skf(2,:),'g-.')

subplot(4,1,3)
plot(tout,q_B_I(3,:),'k')
hold on;grid on;
ylabel('z [-]');
xlabel('time [s]');
plot(tout,q_B_I_f(3,:),'b+-.')
plot(tout,q_B_I_ukf(3,:),'r-.')
plot(tout,q_B_I_skf(3,:),'g-.')

subplot(4,1,4)
plot(tout,q_B_I(4,:),'g')
hold on;grid on;
ylabel('w [-]');
xlabel('time [s]');
plot(tout,q_B_I_f(4,:),'b+-.')
plot(tout,q_B_I_ukf(4,:),'r-.')
plot(tout,q_B_I_skf(4,:),'g-.')

%% ANGULAR RATES MEASUREMENT & ESTIMATE ERROR %%
ymin  = -0.1;
ymax  = 0.1;
textx = 5;
texty = ymax*0.8;

fig = figure;
set(fig,'Position',[screenwidth*(screennumber+0.25) 0 screenwidth*0.25 screenheight]);
subplot(3,1,1)
plot(tout,w_B_BI_m_error(1,:)*rad2deg,'c-.')
hold on;grid on;
plot(tout,w_B_BI_error(1,:)*rad2deg,'b+-.')
plot(tout,w_B_BI_error_ukf(1,:)*rad2deg,'r-.')
plot(tout,w_B_BI_error_skf(1,:)*rad2deg,'g-.')
ylabel('x [\circ/s]');
xlabel('time [s]');
axis([-Inf Inf ymin ymax])
text(textx,texty,strcat('MEAN:',num2str(mean(w_B_BI_error(1,ceil(end/2):end)*rad2deg)),' STD:',num2str(std(w_B_BI_error(1,ceil(end/4):end)*rad2deg))));
title('Angular Estimate Error');

subplot(3,1,2)
plot(tout,w_B_BI_m_error(2,:)*rad2deg,'c-.')
hold on;grid on;
plot(tout,w_B_BI_error(2,:)*rad2deg,'b+-.')
plot(tout,w_B_BI_error_ukf(2,:)*rad2deg,'r-.')
plot(tout,w_B_BI_error_skf(2,:)*rad2deg,'g-.')
ylabel('y [\circ/s]');
xlabel('time [s]');
axis([-Inf Inf ymin ymax])
text(textx,texty,strcat('MEAN:',num2str(mean(w_B_BI_error(2,ceil(end/2):end)*rad2deg)),' STD:',num2str(std(w_B_BI_error(2,ceil(end/4):end)*rad2deg))));


subplot(3,1,3)
plot(tout,w_B_BI_m_error(3,:)*rad2deg,'c-.')
hold on;grid on;
plot(tout,w_B_BI_error(3,:)*rad2deg,'b+-.')
plot(tout,w_B_BI_error_ukf(3,:)*rad2deg,'r-.')
plot(tout,w_B_BI_error_skf(3,:)*rad2deg,'g-.')
ylabel('z [\circ/s]');
xlabel('time [s]');
axis([-Inf Inf ymin ymax])
text(textx,texty,strcat('MEAN:',num2str(mean(w_B_BI_error(3,ceil(end/2):end)*rad2deg)),' STD:',num2str(std(w_B_BI_error(3,ceil(end/4):end)*rad2deg))));

% ANGULAR VELOCITY  TO INERTIAL FRAME
fig = figure;
set(fig,'Position',[screenwidth*(screennumber+0.25) 0 screenwidth*0.25 screenheight]);
ymin = -5;
ymax = 5;

subplot(3,1,1)
plot(tout,w_B_BI_m(1,:)*rad2deg,'c-.');
title('Angular Rates of Body to Inertial Frame')
hold on;grid on;
plot(tout,w_B_BI(1,:)*rad2deg,'k-.')
plot(tout,w_B_BI_f(1,:)*rad2deg,'b+-.')
plot(tout,w_B_BI_ukf(1,:)*rad2deg,'r-.')
plot(tout,w_B_BI_skf(1,:)*rad2deg,'g-.')
ylabel('w_x [\circ/s]');
xlabel('time [s]');
axis([-Inf Inf ymin ymax])

subplot(3,1,2)
plot(tout,w_B_BI_m(2,:)*rad2deg,'c-.')
hold on;grid on;
plot(tout,w_B_BI(2,:)*rad2deg,'k-.')
plot(tout,w_B_BI_f(2,:)*rad2deg,'b+-.')
plot(tout,w_B_BI_ukf(2,:)*rad2deg,'r-.')
plot(tout,w_B_BI_skf(2,:)*rad2deg,'g-.')
ylabel('w_y [\circ/s]');
xlabel('time [s]');
axis([-Inf Inf ymin ymax])

subplot(3,1,3)
plot(tout,w_B_BI_m(3,:)*rad2deg,'c-.')
hold on;grid on;
plot(tout,w_B_BI(3,:)*rad2deg,'k-.')
plot(tout,w_B_BI_f(3,:)*rad2deg,'b+-.')
plot(tout,w_B_BI_ukf(3,:)*rad2deg,'r-.')
plot(tout,w_B_BI_skf(3,:)*rad2deg,'g-.')
ylabel('w_z [\circ/s]');
xlabel('time [s]');
axis([-Inf Inf ymin ymax])

%% BIAS PLOT %%

% Bias
ymin = -0.05;
ymax = 0.05;
textx = 5;
texty = ymax*0.8;

fig = figure;
set(fig,'Position',[screenwidth*(screennumber+0.5) 0 screenwidth*0.25 screenheight]);
subplot(3,1,1)
plot(tout,bias_error(1,:)*rad2deg,'b+-.')
title('Bias Error')
hold on;grid on;
plot(tout,bias_error_ukf(1,:)*rad2deg,'r-.')
plot(tout,bias_error_skf(1,:)*rad2deg,'g-.')
plot(tout,3*sqrt(Pdiag(4,:))*rad2deg,'m--')
plot(tout,-3*sqrt(Pdiag(4,:))*rad2deg,'m--')
ylabel('x [\circ/s]');
xlabel('time [s]');
axis([-Inf Inf ymin ymax])
text(textx,texty,strcat('MEAN:',num2str(mean(bias_error(1,ceil(end/4):end)*rad2deg)),' STD:',num2str(std(bias_error(1,ceil(end/4):end)*rad2deg))));

subplot(3,1,2)
plot(tout,bias_error(2,:)*rad2deg,'b+-.')
hold on;grid on;
plot(tout,bias_error_ukf(2,:)*rad2deg,'r-.')
plot(tout,bias_error_skf(2,:)*rad2deg,'g-.')
plot(tout,3*sqrt(Pdiag(5,:))*rad2deg,'m--')
plot(tout,-3*sqrt(Pdiag(5,:))*rad2deg,'m--')
ylabel('y [\circ/s]');
xlabel('time [s]');
axis([-Inf Inf ymin ymax])
text(textx,texty,strcat('MEAN:',num2str(mean(bias_error(2,ceil(end/4):end)*rad2deg)),' STD:',num2str(std(bias_error(2,ceil(end/4):end)*rad2deg))));

subplot(3,1,3)
plot(tout,bias_error(3,:)*rad2deg,'b+-.')
hold on;grid on;
plot(tout,bias_error_ukf(3,:)*rad2deg,'r-.')
plot(tout,bias_error_skf(3,:)*rad2deg,'g-.')
plot(tout,3*sqrt(Pdiag(6,:))*rad2deg,'m--')
plot(tout,-3*sqrt(Pdiag(6,:))*rad2deg,'m--')
ylabel('z [\circ/s]');
xlabel('time [s]');
axis([-Inf Inf ymin ymax])
text(textx,texty,strcat('MEAN:',num2str(mean(bias_error(3,ceil(end/4):end)*rad2deg)),' STD:',num2str(std(bias_error(3,ceil(end/4):end)*rad2deg))));

% BIAS ESTIMATE
xmin = 0;
xmax = tdur;
ymin = GYRO_Bias*rad2deg-0.1;
ymax = GYRO_Bias*rad2deg+0.1;
fig = figure;
set(fig,'Position',[screenwidth*(screennumber+0.5) 0 screenwidth*0.25 screenheight]);
subplot(3,1,1)
plot(tout,bias(1,:)*rad2deg,'g')
title('Bias Estimate')
hold on;grid on;
ylabel('x [\circ/s]');
xlabel('time [s]');
axis([xmin xmax ymin(1) ymax(1)])
plot(tout,bias_f(1,:)*rad2deg,'b+-.')
plot(tout,bias_ukf(1,:)*rad2deg,'r-.')
plot(tout,bias_skf(1,:)*rad2deg,'g-.')

subplot(3,1,2)
plot(tout,bias(2,:)*rad2deg,'g')
hold on;grid on;
ylabel('y [\circ/s]');
xlabel('time [s]');
axis([xmin xmax ymin(2) ymax(2)])
plot(tout,bias_f(2,:)*rad2deg,'b+-.')
plot(tout,bias_ukf(2,:)*rad2deg,'r-.')
plot(tout,bias_skf(2,:)*rad2deg,'g-.')

subplot(3,1,3)
plot(tout,bias(3,:)*rad2deg,'g')
hold on;grid on;
ylabel('z [\circ/s]');
xlabel('time [s]');
axis([xmin xmax ymin(3) ymax(3)])
plot(tout,bias(3,:)*rad2deg,'b+-.')
plot(tout,bias_ukf(3,:)*rad2deg,'r-.')
plot(tout,bias_skf(3,:)*rad2deg,'g-.')
%% LOS & TORQUE
xmin = 0;
xmax = tdur;
ymin = 0;
ymax = 5000;

fig = figure;
set(fig,'Position',[screenwidth*(screennumber+0.75) 0 screenwidth*0.25 screenheight]);
subplot(3,1,1)
plot(tout,LOS_error_3(1,:),'c-.')
grid on; hold on;
plot(tout,LOS_error(1,:),'b+-.')
plot(tout,LOS_error_ukf(1,:),'r-.')
plot(tout,LOS_error_skf(1,:),'g-.')
title('LOS');
xlabel('time [s]');
ylabel('LOS-X [arcsec]');
axis([xmin xmax ymin ymax])

subplot(3,1,2)
plot(tout,LOS_error_3(2,:),'c-.')
grid on; hold on;
plot(tout,LOS_error(2,:),'b+-.')
plot(tout,LOS_error_ukf(2,:),'r-.')
plot(tout,LOS_error_skf(2,:),'g-.')

xlabel('time [s]');
ylabel('LOS-Y [arcsec]');
axis([xmin xmax ymin ymax])

subplot(3,1,3)
plot(tout,LOS_error_3(3,:),'c-.')

grid on; hold on;
plot(tout,LOS_error(3,:),'b+-.')
plot(tout,LOS_error_ukf(3,:),'r-.')
plot(tout,LOS_error_skf(3,:),'g-.')

xlabel('time [s]');
ylabel('LOS-Z [arcsec]');
axis([xmin xmax ymin ymax])

% TORQUE %%
ymin = -5e-6;
ymax = 5e-6;
fig = figure;
set(fig,'Position',[screenwidth*(screennumber+0.75) 0 screenwidth*0.25 screenheight]);
subplot(3,1,1)
plot(tout,tau_m(1,:),'r')
title('Torque Input')
hold on;grid on;

ylabel('\tau_y [Nm]');
xlabel('time [s]');
axis([-Inf Inf ymin ymax])

subplot(3,1,2)
plot(tout,tau_m(2,:),'r')
hold on;grid on;
ylabel('\tau_y [Nm]');
xlabel('time [s]');
axis([-Inf Inf ymin ymax])

subplot(3,1,3)
plot(tout,tau_m(3,:),'r')
hold on;grid on;
ylabel('\tau_y [Nm]');
xlabel('time [s]');
axis([-Inf Inf ymin ymax])


%% LOS & TORQUE
% xmin = 0;
% xmax = tdur;
% ymin = 0;
% ymax = 100;
% 
% fig = figure;
% set(fig,'Position',[screenwidth*(screennumber+0.75) 0 screenwidth*0.25 screenheight]);
% 
% subplot(3,1,1)
% plot(tout,LOS_SUN)
% axis([xmin xmax ymin ymax])
% grid on; hold on;
% title('LOS');
% xlabel('Period [cycle]');
% ylabel('LOS-Sun [arcsec]');
% 
% 
% subplot(3,1,2)
% plot(tout,LOS_NADIR)
% grid on; hold on;
% xlabel('Period [cycle]');
% ylabel('LOS-Nadir [arcsec]');
% axis([xmin xmax ymin ymax])
% 
% 
% subplot(3,1,3)
% plot(tout,LOS_TGT)
% grid on; hold on;
% xlabel('Period [cycle]');
% ylabel('LOS-Tgt [arcsec]');
% axis([xmin xmax ymin ymax])



