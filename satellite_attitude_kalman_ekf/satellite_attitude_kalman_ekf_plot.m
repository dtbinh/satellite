close all
%% QUATERNION %%
figure
subplot(4,1,1)
plot(tout,q_B_I(1,:),'b')
title('Quaternion Rotation of Body to Inertial Frame')
hold on;grid on;
ylabel('w [-]');
xlabel('time [s]');
plot(tout,q_B_I_m(1,:),'r')
plot(tout,q_B_I_f(1,:),'g')
 
subplot(4,1,2)
plot(tout,q_B_I(2,:),'b')
hold on;grid on;
ylabel('x [-]');
xlabel('time [s]');
plot(tout,q_B_I_m(2,:),'r')
plot(tout,q_B_I_f(2,:),'g')

subplot(4,1,3)
plot(tout,q_B_I(3,:),'b')
hold on;grid on;
ylabel('y [-]');
xlabel('time [s]');
plot(tout,q_B_I_m(3,:),'r')
plot(tout,q_B_I_f(3,:),'g')

subplot(4,1,4)
plot(tout,q_B_I(4,:),'b')
hold on;grid on;
ylabel('z [-]');
xlabel('time [s]');
plot(tout,q_B_I_m(4,:),'r')
plot(tout,q_B_I_f(4,:),'g')


%% EULER ANGLES %%
figure
subplot(3,1,1)
plot(tout,e_B_I_m(1,:)*R2D,'r');
title('Euler Angles ZYX')
hold on;grid on;
ylabel('phi [\circ]');
xlabel('time [s]');
plot(tout,e_B_I(1,:)*R2D,'b');

subplot(3,1,2)
plot(tout,e_B_I_m(2,:)*R2D,'r');
hold on;grid on;
ylabel('theta [\circ]');
xlabel('time [s]');
plot(tout,e_B_I(2,:)*R2D,'b');

subplot(3,1,3)
plot(tout,e_B_I_m(3,:)*R2D,'r');
hold on;grid on;
ylabel('psi [\circ]');
xlabel('time [s]');
plot(tout,e_B_I(3,:)*R2D,'b');

%% ANGULAR VELOCITY %%
figure
subplot(3,1,1)

plot(tout,w_B_BI_m(1,:)*R2D,'r')
title('Angular Rates of Body to Inertial Frame')
hold on;grid on;
plot(tout,w_B_BI_f(1,:)*R2D,'g')
plot(tout,w_B_BI(1,:)*R2D,'b')
ylabel('w_x [\circ/s]');
xlabel('time [s]');

subplot(3,1,2)
plot(tout,w_B_BI_m(2,:)*R2D,'r')
hold on;grid on;
plot(tout,w_B_BI_f(2,:)*R2D,'g')
plot(tout,w_B_BI(2,:)*R2D,'b')
% plot(tout,zeros(length(tout),1),'k');
ylabel('w_y [\circ/s]');
xlabel('time [s]');

subplot(3,1,3)
plot(tout,w_B_BI_m(3,:)*R2D,'r')
hold on;grid on;
plot(tout,w_B_BI_f(3,:)*R2D,'g')
plot(tout,w_B_BI(3,:)*R2D,'b')
ylabel('w_z [\circ/s]');
xlabel('time [s]');

%% GYRO BIAS  %%
figure
subplot(3,1,1)
plot(tout,bias_f(1,:)*R2D,'g')
title(strcat('Angular Rate Bias:',num2str(GYRO_Bias*R2D)))
hold on;grid on;
ylabel('x [\circ/s]');
xlabel('time [s]');

subplot(3,1,2)
plot(tout,bias_f(2,:)*R2D,'g')
hold on;grid on;
ylabel('y [\circ/s]');
xlabel('time [s]');

subplot(3,1,3)
plot(tout,bias_f(3,:)*R2D,'g')
hold on;grid on;
ylabel('z [\circ/s]');
xlabel('time [s]');
%% %%%%%%%%%%%%%%%%%%%%%%%%%% ERROR PLOTS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% QUATERNION ERROR %%
ymin = -0.01;
ymax = 0.01;
textx = 5;
texty = ymax*0.8;

figure
subplot(4,1,1)
plot(tout,q_B_I_error(1,:),'b')
title('Quaternions Error')
hold on;grid on;
ylabel('w [-]');
xlabel('time [s]');
axis([-Inf Inf ymin ymax])
text(textx,texty,strcat('RMS:',num2str(rms(q_B_I_error(1,:)))))

subplot(4,1,2)
plot(tout,q_B_I_error(2,:),'b')
hold on;grid on;
ylabel('x [-]');
xlabel('time [s]');
axis([-Inf Inf ymin ymax])
text(textx,texty,strcat('RMS:',num2str(rms(q_B_I_error(2,:)))))

subplot(4,1,3)
plot(tout,q_B_I_error(3,:),'b')
hold on;grid on;
ylabel('y [-]');
xlabel('time [s]');
axis([-Inf Inf ymin ymax])
text(textx,texty,strcat('RMS:',num2str(rms(q_B_I_error(3,:)))))

subplot(4,1,4)
plot(tout,q_B_I_error(4,:),'b')
hold on;grid on;
ylabel('z [-]');
xlabel('time [s]');
axis([-Inf Inf ymin ymax])
text(textx,texty,strcat('RMS:',num2str(rms(q_B_I_error(4,:)))))

%% EULER ANGLE ERROR %%
ymin = -1;
ymax = 1;
textx = 5;
texty = ymax*0.8;

figure
subplot(3,1,1)
plot(tout,e_B_I_error(1,:)*R2D,'b')
title('Euler Angle Error')
hold on;grid on;
ylabel('x [\circ]');
xlabel('time [s]');
axis([-Inf Inf ymin ymax])
text(textx,texty,strcat('RMS:',num2str(rms(e_B_I_error(1,:)*R2D))))

subplot(3,1,2)
plot(tout,e_B_I_error(2,:)*R2D,'b')
hold on;grid on;
ylabel('y [\circ]');
xlabel('time [s]');
axis([-Inf Inf ymin ymax])
text(textx,texty,strcat('RMS:',num2str(rms(e_B_I_error(2,:)*R2D))))

subplot(3,1,3)
plot(tout,e_B_I_error(3,:)*R2D,'b')
hold on;grid on;
ylabel('z [\circ]');
xlabel('time [s]');
axis([-Inf Inf ymin ymax])
text(textx,texty,strcat('RMS:',num2str(rms(e_B_I_error(3,:)*R2D))))
%% ANGULAR RATES ERROR %%
ymin = -0.5;
ymax = 0.5;
textx = 5;
texty = ymax*0.8;

figure
subplot(3,1,1)
plot(tout,w_B_BI_error(1,:)*R2D,'b')
title('Angular Rates Error')
hold on;grid on;
ylabel('x [\circ/s]');
xlabel('time [s]');
axis([-Inf Inf ymin ymax])
text(textx,texty,strcat('RMS:',num2str(rms(w_B_BI_error(1,:)*R2D))))

subplot(3,1,2)
plot(tout,w_B_BI_error(2,:)*R2D,'b')
hold on;grid on;
ylabel('y [\circ/s]');
xlabel('time [s]');
axis([-Inf Inf ymin ymax])
text(textx,texty,strcat('RMS:',num2str(rms(w_B_BI_error(2,:)*R2D))))

subplot(3,1,3)
plot(tout,w_B_BI_error(3,:)*R2D,'b')
hold on;grid on;
ylabel('z [\circ/s]');
xlabel('time [s]');
axis([-Inf Inf ymin ymax])
text(textx,texty,strcat('RMS:',num2str(rms(w_B_BI_error(3,:)*R2D))))
%% BIAS ERROR %%
ymin = -0.01;
ymax = 0.01;
textx = 5;
texty = ymax*0.8;

figure
subplot(3,1,1)
plot(tout,bias_error(1,:)*R2D,'b')
title('Bias Error')
hold on;grid on;
ylabel('x [\circ/s]');
xlabel('time [s]');
axis([-Inf Inf ymin ymax])
text(textx,texty,strcat('RMS:',num2str(rms(bias_error(1,end/2:end)*R2D))))

subplot(3,1,2)
plot(tout,bias_error(2,:)*R2D,'b')
hold on;grid on;
ylabel('y [\circ/s]');
xlabel('time [s]');
axis([-Inf Inf ymin ymax])
text(textx,texty,strcat('RMS:',num2str(rms(bias_error(2,end/2:end)*R2D))))

subplot(3,1,3)
plot(tout,bias_error(3,:)*R2D,'b')
hold on;grid on;
ylabel('z [\circ/s]');
xlabel('time [s]');
axis([-Inf Inf ymin ymax])
text(textx,texty,strcat('RMS:',num2str(rms(bias_error(3,end/2:end)*R2D))))