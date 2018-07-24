close all
clear all
clc
format long

model_type = 'BST_RW_MODEL_TYPE_GENERAL_BLDC';
ctrl_mode  = 'BST_RW_CTRL_MODE_TORQUE';

% IADCS  SPEED CONTROL PROPAGATION MODEL PARAMETERS
model_k = 1.000000000000000;
model_d = 2.784230231948520;
model_t = 0.556846046389705;

% CONTROL MODE TORQUE
kp = 76; 
ki = 477; 
kd = 0.0;

% 
bldc_ke = 0.002115/(2*pi/60); % [V/rad/s]
bldc_R  = 8.6;                % [Ohm]
bldc_n  = 0.77;               % [-] Efficiency
bldc_km = 0.0202;             % [Nm/A]

U_sup = 23.1;   %[V]

frict_a = 5.69935E-4;                      % [Nm] 
frict_b = 2.06636E-7/(2*pi/60);            % [Nm/(rad/s)]
frict_c = 1.58979E-11/(2*pi/60)/(2*pi/60); % [Nm/(rad/s)^2]

% Speed control
wtgt = 1000/60*2*pi; %[rad/s]
wmax = 10000/60*2*pi;

% Torque Control
Ttgt = 0.01;    %[Nm]

% Current control
itgt = 0.5; % [A]
imax = 0.82; % [A]


dt   = 0.1;
tdur = 30;
t    = 0:dt:tdur;

% MEMORY ALLOCATION
T    = zeros(tdur/dt+1,1);
h    = zeros(tdur/dt+1,1);
w    = zeros(tdur/dt+1,1);
cur  = zeros(tdur/dt+1,1);

Tdot = zeros(tdur/dt+1,1);
hdot = zeros(tdur/dt+1,1);
wdot = zeros(tdur/dt+1,1);
Tfric= zeros(tdur/dt+1,1);

Tm    = zeros(tdur/dt+1,1);
wm    = zeros(tdur/dt+1,1);

Ttgt = Ttgt*ones(tdur/dt+1,1);
wtgt = wtgt*ones(tdur/dt+1,1);
itgt = zeros(tdur/dt+1,1);

w(1) = 0; % Initial Angular Velocity
h(1) = 0;
T(1) = 0;
hdot(1) = 0;
init = 1;

for i = 2:1:tdur/dt+1
 if strcmp(model_type ,'BST_RW_MODEL_TYPE_GENERAL_BLDC')
     
     I = 0.0009785480;
     
     if init == 1
         fprintf('BST_RW_MODEL_TYPE_GENERAL_BLDC\n');
         in_prev = 0;
         init = 0;
     else
         in_prev = Ttgt(i-1) - Tm(i-2);
     end
     
     % PID Control
     pid_kp = 22; 
     pid_ki = 0.2; 
     pid_kd = 0.0;
     
     in      = Ttgt(i-1) - Tm(i-1);
     itgt(i) = pid_kp * in + pid_ki * (in*dt) + pid_kd * ((in - in_prev)/dt);
     
     % Dynamics
     cur(i)     = sat(itgt(i),imax); 
     i_max_bemf = (U_sup-abs(w(i-1)*bldc_ke))/bldc_R;
     cur(i)     = sat(cur(i),i_max_bemf);
     
     Tfric(i) = -sign(w(i-1))*(frict_c*w(i-1)^2 + frict_b*abs(w(i-1)) + frict_a);
     T(i)  = bldc_n*bldc_km*cur(i) + Tfric(i);
     h(i)  = h(i-1)+T(i)*dt;
     w(i)  = 1/I*h(i);    
     
     % Sensor
     Tm(i) = T(i);
     wm(i) = w(i);
 end
 
 if strcmp(model_type ,'BST_RW_MODEL_TYPE_HT_RW200_15')
     
     I = 1.546500e-06;

     if strcmp(ctrl_mode ,'BST_RW_CTRL_MODE_TORQUE')
         if init == 1
             fprintf('BST_RW_RW200_15_CTRL_MODE_TORQUE\n');

             init = 0;
         end
         
         
         wdot(i) = Ttgt(i)/I;
         hdot(i) = I*wdot(i);
         T(i) = hdot(i);
         h(i) = h(i-1)+hdot(i)*dt;
         w(i) = w(i-1)+wdot(i)*dt;
         
     elseif strcmp(ctrl_mode ,'BST_RW_CTRL_MODE_SPEED')
         if init == 1
         fprintf('BST_RW_RW200_15_CTRL_MODE_SPEED\n');
         init = 0;
         end
         
         Tdot(i) = I*(-2*model_d/(I*model_t)*hdot(i-1) + 1/(model_t^2)*(model_k*wtgt(i)-w(i-1)));   
         dh = Tdot(i)*dt;
         T(i) = T(i-1)+dh;
         h(i) = h(i-1)+T(i)*dt;
         w(i) = 1/I*h(i);
         hdot(i)= T(i);
     end
 end
 
end


figure
plot(t,w,'x-')
grid on
xlabel('Time [s]');
ylabel('Wheel Angular Velocity [rad/s]'); 

figure
plot(t,T,'x-')
grid on
xlabel('Time [s]');
ylabel('Torque [Nm]'); 

figure
plot(t,cur,'x-')
grid on
xlabel('Time [s]');
ylabel('Current [A]'); 

figure
plot(t,Tfric,'x-')
grid on
xlabel('Time [s]');
ylabel('Friction [Nm]'); 