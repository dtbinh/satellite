close all
screensize   = get(0,'ScreenSize');
screenwidth  = screensize(3);
screenheight = screensize(4);
screennumber = 0;

%% QUATERNION ERROR %%
ymin = -10*std(q_B_I_error(1,ceil(end/2):end));
ymax = 10*std(q_B_I_error(1,ceil(end/2):end));
textx = 5;
texty = ymax*0.8;

fig = figure;
set(fig,'Position',[screenwidth*(screennumber+0) 0 screenwidth*0.25 screenheight]);

subplot(8,1,1)
plot(tout,q_B_I_error(1,:),'b')
title('QUATERNION ERROR')
hold on;grid on;
ylabel('x [-]');
xlabel('time [s]');
axis([-Inf Inf ymin ymax])
text(textx,texty,strcat('MEAN:',num2str(mean(q_B_I_error(1,ceil(end/4):end))),' STD:',num2str(std(q_B_I_error(1,ceil(end/4):end)))));

subplot(8,1,2)
plot(tout,q_B_I_error(2,:),'b')
hold on;grid on;
ylabel('y [-]');
xlabel('time [s]');
axis([-Inf Inf ymin ymax])
text(textx,texty,strcat('MEAN:',num2str(mean(q_B_I_error(2,ceil(end/4):end))),' STD:',num2str(std(q_B_I_error(2,ceil(end/4):end)))));

subplot(8,1,3)
plot(tout,q_B_I_error(3,:),'b')
hold on;grid on;
ylabel('z [-]');
xlabel('time [s]');
axis([-Inf Inf ymin ymax])
text(textx,texty,strcat('MEAN:',num2str(mean(q_B_I_error(3,ceil(end/4):end))),' STD:',num2str(std(q_B_I_error(3,ceil(end/4):end)))));

subplot(8,1,4)
plot(tout,q_B_I_error(4,:),'b')
hold on;grid on;
ylabel('w [-]');
xlabel('time [s]');
axis([-Inf Inf ymin ymax])
text(textx,texty,strcat('MEAN:',num2str(mean(q_B_I_error(4,ceil(end/4):end))),' STD:',num2str(std(q_B_I_error(4,ceil(end/4):end)))));

% QUATERNION TO INERTIAL FRAME 
subplot(8,1,5)
plot(tout,q_B_I(1,:),'b')
title('Quaternion Rotation of Body to Inertial Frame')
hold on;grid on;
ylabel('x [-]');
xlabel('time [s]');
plot(tout,q_B_I_m_1(1,:),'r')
plot(tout,q_B_I_m_2(1,:),'r')
plot(tout,q_B_I_f(1,:),'g')
 
subplot(8,1,6)
plot(tout,q_B_I(2,:),'b')
hold on;grid on;
ylabel('y [-]');
xlabel('time [s]');
plot(tout,q_B_I_m_1(2,:),'r')
plot(tout,q_B_I_m_2(2,:),'r')
plot(tout,q_B_I_f(2,:),'g')

subplot(8,1,7)
plot(tout,q_B_I(3,:),'b')
hold on;grid on;
ylabel('z [-]');
xlabel('time [s]');
plot(tout,q_B_I_m_1(3,:),'r')
plot(tout,q_B_I_m_2(3,:),'r')
plot(tout,q_B_I_f(3,:),'g')

subplot(8,1,8)
plot(tout,q_B_I(4,:),'b')
hold on;grid on;
ylabel('w [-]');
xlabel('time [s]');
plot(tout,q_B_I_m_1(4,:),'r')
plot(tout,q_B_I_m_2(4,:),'r')
plot(tout,q_B_I_f(4,:),'g')


%% ANGULAR RATES MEASUREMENT & ESTIMATE ERROR %%
ymin  = -0.5;
ymax  = 0.5;
textx = 5;
texty = ymax*0.8;

fig = figure;
set(fig,'Position',[screenwidth*(screennumber+0.25) 0 screenwidth*0.25 screenheight]);
subplot(6,1,1)
plot(tout,w_B_BI_m_error(1,:)*R2D,'b')
hold on;grid on;
plot(tout,w_B_BI_error(1,:)*R2D,'g')
ylabel('x [\circ/s]');
xlabel('time [s]');
axis([-Inf Inf ymin ymax])
text(textx,texty,strcat('MEAN:',num2str(mean(w_B_BI_error(1,ceil(end/2):end)*R2D)),' STD:',num2str(std(w_B_BI_error(1,ceil(end/4):end)*R2D))));
title('ANGULAR RATES MEASUREMENT & ESTIMATE ERROR');

subplot(6,1,2)
plot(tout,w_B_BI_m_error(2,:)*R2D,'b')
hold on;grid on;
plot(tout,w_B_BI_error(2,:)*R2D,'g')
ylabel('y [\circ/s]');
xlabel('time [s]');
axis([-Inf Inf ymin ymax])
text(textx,texty,strcat('MEAN:',num2str(mean(w_B_BI_error(2,ceil(end/2):end)*R2D)),' STD:',num2str(std(w_B_BI_error(2,ceil(end/4):end)*R2D))));


subplot(6,1,3)
plot(tout,w_B_BI_m_error(3,:)*R2D,'b')
hold on;grid on;
plot(tout,w_B_BI_error(3,:)*R2D,'g')
ylabel('z [\circ/s]');
xlabel('time [s]');
axis([-Inf Inf ymin ymax])
text(textx,texty,strcat('MEAN:',num2str(mean(w_B_BI_error(3,ceil(end/2):end)*R2D)),' STD:',num2str(std(w_B_BI_error(3,ceil(end/4):end)*R2D))));

% ANGULAR VELOCITY  TO INERTIAL FRAME
ymin = -5;
ymax = 5;

subplot(6,1,4)
plot(tout,w_B_BI_m(1,:)*R2D,'r')
title('Angular Rates of Body to Inertial Frame')
hold on;grid on;
plot(tout,w_B_BI_f(1,:)*R2D,'g')
plot(tout,w_B_BI(1,:)*R2D,'b')

ylabel('w_x [\circ/s]');
xlabel('time [s]');
axis([-Inf Inf ymin ymax])

subplot(6,1,5)
plot(tout,w_B_BI_m(2,:)*R2D,'r')
hold on;grid on;
plot(tout,w_B_BI_f(2,:)*R2D,'g')
plot(tout,w_B_BI(2,:)*R2D,'b')

% plot(tout,zeros(length(tout),1),'k');
ylabel('w_y [\circ/s]');
xlabel('time [s]');
axis([-Inf Inf ymin ymax])

subplot(6,1,6)
plot(tout,w_B_BI_m(3,:)*R2D,'r')
hold on;grid on;
plot(tout,w_B_BI_f(3,:)*R2D,'g')
plot(tout,w_B_BI(3,:)*R2D,'b')

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
subplot(6,1,1)
plot(tout,bias_error(1,:)*R2D,'b')
title('Bias')
hold on;grid on;
plot(tout,3*sqrt(Pdiag(4,:))*R2D,'r')
plot(tout,-3*sqrt(Pdiag(4,:))*R2D,'r')
ylabel('x [\circ/s]');
xlabel('time [s]');
axis([-Inf Inf ymin ymax])
text(textx,texty,strcat('MEAN:',num2str(mean(bias_error(1,ceil(end/4):end)*R2D)),' STD:',num2str(std(bias_error(1,ceil(end/4):end)*R2D))));

subplot(6,1,2)
plot(tout,bias_error(2,:)*R2D,'b')
hold on;grid on;
plot(tout,3*sqrt(Pdiag(5,:))*R2D,'r')
plot(tout,-3*sqrt(Pdiag(5,:))*R2D,'r')
ylabel('y [\circ/s]');
xlabel('time [s]');
axis([-Inf Inf ymin ymax])
text(textx,texty,strcat('MEAN:',num2str(mean(bias_error(2,ceil(end/4):end)*R2D)),' STD:',num2str(std(bias_error(2,ceil(end/4):end)*R2D))));

subplot(6,1,3)
plot(tout,bias_error(3,:)*R2D,'b')
hold on;grid on;
plot(tout,3*sqrt(Pdiag(6,:))*R2D,'r')
plot(tout,-3*sqrt(Pdiag(6,:))*R2D,'r')
ylabel('z [\circ/s]');
xlabel('time [s]');
axis([-Inf Inf ymin ymax])
text(textx,texty,strcat('MEAN:',num2str(mean(bias_error(3,ceil(end/4):end)*R2D)),' STD:',num2str(std(bias_error(3,ceil(end/4):end)*R2D))));

% BIAS ESTIMATE
xmin = 0;
xmax = tdur;
ymin = -inf;
ymax = inf;
textx = 5;
texty = ymax*0.8;


subplot(6,1,4)
plot(tout,bias_f(1,:)*R2D,'b')
title('Bias Estimate')
hold on;grid on;
ylabel('x [\circ/s]');
xlabel('time [s]');
axis([xmin xmax ymin ymax])
plot(tout,bias(1,:)*R2D,'r')

subplot(6,1,5)
plot(tout,bias_f(2,:)*R2D,'b')
hold on;grid on;
ylabel('y [\circ/s]');
xlabel('time [s]');
axis([xmin xmax ymin ymax])
plot(tout,bias(2,:)*R2D,'r')

subplot(6,1,6)
plot(tout,bias_f(3,:)*R2D,'b')
hold on;grid on;
ylabel('z [\circ/s]');
xlabel('time [s]');
axis([xmin xmax ymin ymax])
plot(tout,bias(3,:)*R2D,'r')

%% LOS & TORQUE
xmin = 0;
xmax = tdur;
ymin = 0;
ymax = 500;

fig = figure;
set(fig,'Position',[screenwidth*(screennumber+0.75) 0 screenwidth*0.25 screenheight]);
subplot(6,1,1)
plot(tout,LOS_error(1,:))
axis([xmin xmax ymin ymax])
grid on; hold on;
title('LOS');
xlabel('Period [cycle]');
ylabel('LOS-Sun [arcsec]');

subplot(6,1,2)
plot(tout,LOS_error(2,:))
grid on; hold on;
xlabel('Period [cycle]');
ylabel('LOS-Nadir [arcsec]');
axis([xmin xmax ymin ymax])

subplot(6,1,3)
plot(tout,LOS_error(3,:))
grid on; hold on;
xlabel('Period [cycle]');
ylabel('LOS-Tgt [arcsec]');
axis([xmin xmax ymin ymax])




% TORQUE %%
ymin = -5e-6;
ymax = 5e-6;

subplot(6,1,4)
plot(tout,tau_m(1,:),'r')
title('Torque Input')
hold on;grid on;

ylabel('\tau_y [Nm]');
xlabel('time [s]');
axis([-Inf Inf ymin ymax])

subplot(6,1,5)
plot(tout,tau_m(2,:),'r')
hold on;grid on;
ylabel('\tau_y [Nm]');
xlabel('time [s]');
axis([-Inf Inf ymin ymax])

subplot(6,1,6)
plot(tout,tau_m(3,:),'r')
hold on;grid on;
ylabel('\tau_y [Nm]');
xlabel('time [s]');
axis([-Inf Inf ymin ymax])


%% LOS & TORQUE
xmin = 0;
xmax = tdur;
ymin = 0;
ymax = 100;

fig = figure;
set(fig,'Position',[screenwidth*(screennumber+0.75) 0 screenwidth*0.25 screenheight]);

subplot(3,1,1)
plot(tout,LOS_SUN)
axis([xmin xmax ymin ymax])
grid on; hold on;
title('LOS');
xlabel('Period [cycle]');
ylabel('LOS-Sun [arcsec]');


subplot(3,1,2)
plot(tout,LOS_NADIR)
grid on; hold on;
xlabel('Period [cycle]');
ylabel('LOS-Nadir [arcsec]');
axis([xmin xmax ymin ymax])


subplot(3,1,3)
plot(tout,LOS_TGT)
grid on; hold on;
xlabel('Period [cycle]');
ylabel('LOS-Tgt [arcsec]');
axis([xmin xmax ymin ymax])



